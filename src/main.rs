//! Firmware for rising tide CAN-IMU
//!
//! The board provides roll, pitch and heave data on a NMEA2000 bus, and also support CANOpen for
//! configuration. It supports two IMUs: an expensive Microstrain 3DM-CV7-AHRS, and a cheap
//! Invensense ICM20948 for comparison.
#![no_std]
#![no_main]

use cortex_m_rt::{self as _};
use defmt::info;
use embedded_hal::delay::DelayNs;
use embedded_io::Read;
use panic_probe as _;
use rtt_target::{ChannelMode, rtt_init, set_defmt_channel};
use zencan_node::{
    Callbacks, Node, common::NodeId, object_dict::ODEntry, restore_stored_comm_objects,
    restore_stored_objects,
};

use core::{
    cell::RefCell,
    convert::Infallible,
    hash::Hasher as _,
    pin::pin,
    sync::atomic::{AtomicUsize, Ordering},
    time::Duration,
};

use hash32::FnvHasher;
use lilos::exec::{Interrupts, Notify};
use stm32_hal2::{
    self as hal,
    gpio::{Edge, Pin, PinMode, Port},
    pac::interrupt,
    usart::UsartConfig,
};

use stm32_metapac as pac;

use rjmp_stm32_flash::Stm32gxPagePair;

/// can module wires up CAN message sending and receiving
mod can;
// 3DM-CM7 IMU support
mod cm7_imu;
/// Flash storage utiltiies / definitions
mod flash;
/// ICM20948 IMU support
mod icm_imu;
/// state estimation algorithms
mod state;

use flash::*;

// Instantiate zencan static objects
pub mod zencan {
    zencan_node::include_modules!(ZENCAN_CONFIG);
}

/// A notification for the ICM20948 IRQ
static ICM_NOTIFY: Notify = Notify::new();

/// A notification used to wake the zencan process
static CAN_NOTIFY: Notify = Notify::new();

#[allow(static_mut_refs)]
static mut UART: Option<hal::usart::Usart<hal::pac::USART1>> = None;

static CM7_IMU: microstrain_inertial::interface::AsyncInterface =
    microstrain_inertial::interface::AsyncInterface::new();

/// Callback for zencan to notify task that there are messages to be processed
fn notify_can_task() {
    CAN_NOTIFY.notify();
}

/// Compute a 32-bit unique serial number from the UID register value
fn get_serial() -> u32 {
    let mut ctx: FnvHasher = Default::default();
    ctx.write(&pac::UID.uid(0).read().to_le_bytes());
    ctx.write(&pac::UID.uid(1).read().to_le_bytes());
    ctx.write(&pac::UID.uid(2).read().to_le_bytes());
    let digest = ctx.finish();
    digest as u32
}

/// A simple delay object which busy loops on the lilos systick timer
struct Delay {}
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        let start = lilos::time::TickTime::now();
        while start.elapsed().0 * 1000000 < ns as u64 {}
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    // Setup RTT for defmt output
    let channels = rtt_init! {
        up: {
            0: {
                size: 2048,
                mode: ChannelMode::NoBlockSkip,
                name: "defmt"
            }
        }
    };
    set_defmt_channel(channels.up.0);

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::pac::Peripherals::take().unwrap();
    let clock_cfg = hal::clocks::Clocks::default();
    clock_cfg.setup().unwrap();

    info!("Sysclk: {}", clock_cfg.sysclk());
    info!("APB1: {}", clock_cfg.apb1());
    info!("APB2: {}", clock_cfg.apb2());

    // Enable clocks
    pac::RCC.apb1enr1().modify(|w| w.set_fdcanen(true));
    pac::RCC.apb1smenr1().modify(|w| w.set_fdcansmen(true));
    pac::RCC
        .ccipr()
        .modify(|w| w.set_fdcansel(stm32_metapac::rcc::vals::Fdcansel::PCLK1));
    pac::DBGMCU.cr().modify(|w| {
        w.set_dbg_standby(true);
        w.set_dbg_stop(true);
    });
    // Have to enable DMA1 clock to keep RAM accessible for RTT during debug
    pac::RCC.ahb1enr().modify(|w| w.set_dma1en(true));

    pac::RCC.ahb2enr().modify(|w| w.set_gpioben(true));
    // Setup CAN pins for FDCAN1 alternate function
    let _can_tx = Pin::new(Port::B, 9, PinMode::Alt(9));
    let _can_rx = Pin::new(Port::B, 8, PinMode::Alt(9));
    // Enable the buffer between the CAN RX output and the MCU The buffer is there because the pin
    // doubles as BOOT0 during power-on, and is also connected to the DIP switch to enable
    // bootloader
    let mut can_rx_en = Pin::new(Port::B, 7, PinMode::Output);
    can_rx_en.set_high();

    // CAN Transceiver put into normal operating mode
    let mut can_stby = Pin::new(Port::B, 6, PinMode::Output);
    can_stby.set_low();
    let mut can_shdn = Pin::new(Port::B, 4, PinMode::Output);
    can_shdn.set_low();

    // Initialize the CAN peripheral. Message passing between FDCAN and zencan is done in IRQ.
    can::init_can();

    // setup 3DM uart pins for Uart1
    let _3dm_rx = Pin::new(Port::A, 10, PinMode::Alt(7));
    let _3dm_tx = Pin::new(Port::A, 9, PinMode::Alt(7));

    let mut uart =
        hal::usart::Usart::new(dp.USART1, 115200, UsartConfig::default(), &clock_cfg).unwrap();
    uart.enable().unwrap();
    uart.enable_interrupt(hal::usart::UsartInterrupt::TransmitEmpty)
        .unwrap();
    uart.enable_interrupt(hal::usart::UsartInterrupt::ReadNotEmpty)
        .unwrap();
    unsafe { UART = Some(uart) };

    // Setup ICM IMU pins on SPI1
    let _icm_sdi = Pin::new(Port::A, 7, PinMode::Alt(5));
    let _icm_sdo = Pin::new(Port::A, 6, PinMode::Alt(5));
    let _icm_sck = Pin::new(Port::A, 5, PinMode::Alt(5));
    let mut icm_cs = Pin::new(Port::B, 0, PinMode::Output);
    icm_cs.set_high();

    let mut icm_int1 = Pin::new(Port::B, 1, PinMode::Input);
    icm_int1.enable_interrupt(Edge::Rising);

    // Read the NMEA mode input pin, and store its value. It is only read at boot
    let nmea_cfg_pin = Pin::new(Port::A, 4, PinMode::Input);
    if nmea_cfg_pin.is_high() {
        zencan::OBJECT2200.set_nmea_enabled(1);
    } else {
        zencan::OBJECT2200.set_nmea_enabled(0);
    }

    let persist_flash = RefCell::new(Stm32gxPagePair::new(60, 62, 2));

    let mut store_node_config = |node_id: NodeId| {
        let mut flash = persist_flash.borrow_mut();
        store_node_config(&mut *flash, node_id);
    };
    let mut store_objects = |reader: &mut dyn Read<Error = Infallible>, len: usize| {
        let mut flash = persist_flash.borrow_mut();
        store_objects(&mut *flash, reader, len);
    };
    let mut reset_app = |od: &[ODEntry]| {
        let mut flash = persist_flash.borrow_mut();
        // On RESET APP transition, we reload object values to their reset value

        // Restore objects saved to flash
        read_persisted_objects(&mut *flash, |stored_data| {
            restore_stored_objects(od, stored_data)
        });
    };
    let mut reset_comms = |od: &[ODEntry]| {
        let mut flash = persist_flash.borrow_mut();
        // On reset COMMS, only the communications objects (0x1000-0x1fff) are restored. The node
        // library will handle restoring the default values before calling the reset_comms callback.
        // Then the application may restore objects from persistent storage if it supports that.
        read_persisted_objects(&mut *flash, |stored_data| {
            restore_stored_comm_objects(od, stored_data)
        });
    };

    //
    // Setup zencan node
    //
    zencan::OBJECT1018.set_serial(get_serial());

    let mut callbacks = Callbacks::default();
    callbacks.store_node_config = Some(&mut store_node_config);
    callbacks.store_objects = Some(&mut store_objects);
    callbacks.reset_app = Some(&mut reset_app);
    callbacks.reset_comms = Some(&mut reset_comms);

    let node = Node::new(
        NodeId::new(10).unwrap(),
        callbacks,
        &zencan::NODE_MBOX,
        &zencan::NODE_STATE,
        &zencan::OD_TABLE,
    );

    // Register handler for waking process task
    zencan::NODE_MBOX.set_process_notify_callback(&notify_can_task);

    // Register handler for CAN frame transmit notice
    zencan::NODE_MBOX.set_transmit_notify_callback(&can::transmit_notify_handler);

    unsafe { cortex_m::interrupt::enable() };
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI1) };
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART1) };

    lilos::time::initialize_sys_tick(&mut cp.SYST, clock_cfg.systick());

    let spi_config = rjmp_stm32_spi::SpiConfig {
        data_size: rjmp_stm32_spi::DataSize::D8,
        mode: rjmp_stm32_spi::MODE_0,
        clock_div: rjmp_stm32_spi::ClockDiv::Div32,
        bit_order: rjmp_stm32_spi::BitOrder::MsbFirst,
    };
    let spi = rjmp_stm32_spi::Spi::spi1(spi_config);

    defmt::info!("Starting tasks");
    unsafe {
        lilos::exec::run_tasks_with_preemption(
            &mut [
                pin!(can_task(node)),
                pin!(icm_imu::imu_task(spi, icm_cs)),
                pin!(cm7_imu::cm7_task()),
            ],
            lilos::exec::ALL_TASKS,
            Interrupts::Filtered(0xFF),
        )
    }
}

/// A task for running the CAN node processing periodically, or when triggered by the CAN receive
/// interrupt to run immediately
async fn can_task(mut node: Node<'_>) -> Infallible {
    let epoch = lilos::time::TickTime::now();
    loop {
        lilos::time::with_timeout(Duration::from_millis(50), CAN_NOTIFY.until_next()).await;
        let time_us = epoch.elapsed().0 * 1000;
        node.process(time_us);
    }
}

#[interrupt]
fn EXTI1() {
    hal::gpio::clear_exti_interrupt(1);
    ICM_NOTIFY.notify();
}

// #[allow(static_mut_refs)]
// static mut CM7_FRAMER: microstrain_inertial::framer::MessageFramer;

static CM7_FRAMER_ERROR_COUNT: AtomicUsize = AtomicUsize::new(0);

fn enable_uart_tx_irq() {
    pac::USART1.cr1().modify(|w| w.set_txeie(true));
}

#[allow(static_mut_refs)]
#[interrupt]
fn USART1() {
    static mut CM7_FRAMER: microstrain_inertial::framer::MessageFramer =
        microstrain_inertial::framer::MessageFramer::new();

    // Safety: Only access the UART in this IRQ
    let uart = unsafe { UART.as_mut().unwrap() };

    if uart.check_status_flag(stm32_hal2::usart::UsartInterrupt::ReadNotEmpty) {
        let byte = uart.read_one();
        match CM7_FRAMER.push_byte(byte) {
            Ok(Some(packet)) => CM7_IMU.push_message(packet),
            Ok(None) => (),
            Err(_) => {
                CM7_FRAMER_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            }
        };
    }

    if uart.check_status_flag(stm32_hal2::usart::UsartInterrupt::TransmitEmpty) {
        let mut buf = [0];
        let cnt = CM7_IMU.read_outgoing_bytes(&mut buf);
        if cnt > 0 {
            uart.write_one(buf[0]);
        } else {
            uart.disable_interrupt(hal::usart::UsartInterrupt::TransmitEmpty);
        }
    }
}
