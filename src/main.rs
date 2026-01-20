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
use panic_probe as _;
use rtt_target::{rtt_init, set_defmt_channel};
use zencan_node::{
    Callbacks, Node,
    common::{NodeId, sdo::AbortCode},
    object_dict::ObjectAccess,
};

use num_traits::float::Float;

use core::{convert::Infallible, f32, hash::Hasher as _, pin::pin, time::Duration};

use hash32::FnvHasher;
use lilos::exec::{Interrupts, Notify};
use stm32_hal2::{
    self as hal,
    gpio::{OutputType, Pin, PinMode, Port},
    usart::UsartConfig,
};

use icm20948::{
    AccelDataG, AccelDlpf, AccelFullScale, GyroDataRps, GyroDlpf, GyroFullScale, InterruptConfig,
    MagConfig, MagDataUT, SpiInterface,
    sensors::{AccelConfig, GyroConfig},
};

use stm32_metapac as pac;

/// can module wires up CAN message sending and receiving
mod can;

// Instantiate zencan static objects
mod zencan {
    zencan_node::include_modules!(ZENCAN_CONFIG);
}

/// A notification used to wake the zencan process
static CAN_NOTIFY: Notify = Notify::new();

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
                size: 512,
                name: "defmt",
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
    // Setup CAN pins
    let _can_tx = Pin::new(Port::B, 9, PinMode::Alt(9));
    let _can_rx = Pin::new(Port::B, 8, PinMode::Alt(9));
    let mut can_rx_en = Pin::new(Port::B, 7, PinMode::Output);

    can_rx_en.output_type(OutputType::PushPull);
    let mut can_stby = Pin::new(Port::B, 6, PinMode::Output);
    can_stby.set_low();
    let mut can_shdn = Pin::new(Port::B, 4, PinMode::Output);
    can_shdn.set_low();

    can_rx_en.set_high();

    info!("CAN Init");
    can::init_can();

    // setup 3DM uart pins
    let _3dm_rx = Pin::new(Port::A, 10, PinMode::Alt(7));
    let _3dm_tx = Pin::new(Port::A, 9, PinMode::Alt(7));

    // Setup ICM IMU SPI pins
    let _icm_sdi = Pin::new(Port::A, 7, PinMode::Alt(5));
    let _icm_sdo = Pin::new(Port::A, 6, PinMode::Alt(5));
    let _icm_sck = Pin::new(Port::A, 5, PinMode::Alt(5));
    let mut icm_cs = Pin::new(Port::B, 0, PinMode::Output);
    icm_cs.set_high();
    let _icm_int1 = Pin::new(Port::B, 1, PinMode::Input);

    let _uart =
        hal::usart::Usart::new(dp.USART1, 115200, UsartConfig::default(), &clock_cfg).unwrap();

    //
    // Setup zencan node
    //
    zencan::OBJECT1018.set_serial(get_serial());
    let callbacks = Callbacks {
        store_node_config: None,
        store_objects: None,
        reset_app: None,
        reset_comms: None,
        enter_operational: None,
        enter_stopped: None,
        enter_preoperational: None,
    };
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

    lilos::time::initialize_sys_tick(&mut cp.SYST, clock_cfg.systick());

    let spi_config = rjmp_stm32_spi::SpiConfig {
        data_size: rjmp_stm32_spi::DataSize::D8,
        mode: rjmp_stm32_spi::MODE_0,
        clock_div: rjmp_stm32_spi::ClockDiv::Div32,
        bit_order: rjmp_stm32_spi::BitOrder::MsbFirst,
    };
    let spi = rjmp_stm32_spi::Spi::spi1(spi_config);

    unsafe {
        lilos::exec::run_tasks_with_preemption(
            &mut [pin!(can_task(node)), pin!(imu_task(spi, icm_cs))],
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

#[inline]
/// Function to store IMU data to object dict
/// 
/// Mainly motivated by the belief that it will create less panic sites and therefore smaller code
/// although this is an untested theory
fn store_icm20948_values(
    gyro: GyroDataRps,
    accel: AccelDataG,
    mag: MagDataUT,
) -> Result<(), AbortCode> {
    const GYRO_SCALE: f32 = 32768.0 / 1000.0 * core::f32::consts::PI / 180.0; // LSB per deg/s
    const ACCEL_SCALE: f32 = 32768.0 / 8.0; // LSB per g
    const MAG_SCALE: f32 = 1.0 / 0.015; // LSB per uT
    zencan::OBJECT2000.set(0, (gyro.x * GYRO_SCALE).round() as i16)?;
    zencan::OBJECT2000.set(1, (gyro.y * GYRO_SCALE).round() as i16)?;
    zencan::OBJECT2000.set(2, (gyro.z * GYRO_SCALE).round() as i16)?;
    zencan::OBJECT2001.set(0, gyro.x)?;
    zencan::OBJECT2001.set(1, gyro.y)?;
    zencan::OBJECT2001.set(2, gyro.z)?;

    zencan::OBJECT2002.set(0, (accel.x * ACCEL_SCALE).round() as i16)?;
    zencan::OBJECT2002.set(1, (accel.y * ACCEL_SCALE).round() as i16)?;
    zencan::OBJECT2002.set(2, (accel.z * ACCEL_SCALE).round() as i16)?;
    zencan::OBJECT2003.set(0, accel.x)?;
    zencan::OBJECT2003.set(1, accel.y)?;
    zencan::OBJECT2003.set(2, accel.z)?;

    zencan::OBJECT2004.set(0, (mag.x * MAG_SCALE).round() as i16)?;
    zencan::OBJECT2004.set(1, (mag.y * MAG_SCALE).round() as i16)?;
    zencan::OBJECT2004.set(2, (mag.z * MAG_SCALE).round() as i16)?;
    zencan::OBJECT2005.set(0, mag.x)?;
    zencan::OBJECT2005.set(1, mag.y)?;
    zencan::OBJECT2005.set(2, mag.z)?;

    for i in 0..3 {
        zencan::OBJECT2000.set_event_flag(i)?;
        zencan::OBJECT2001.set_event_flag(i)?;
        zencan::OBJECT2002.set_event_flag(i)?;
        zencan::OBJECT2003.set_event_flag(i)?;
        zencan::OBJECT2004.set_event_flag(i)?;
        zencan::OBJECT2005.set_event_flag(i)?;
    }
    Ok(())
}

/// A task for reading data from the IMU
async fn imu_task<S: embedded_hal::spi::SpiBus>(spi: S, cs: Pin) -> Infallible {
    let device = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let mut imu = icm20948::Icm20948Driver::new(SpiInterface::new(device)).unwrap();
    imu.init(&mut Delay {}).unwrap();

    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G8,
        dlpf: AccelDlpf::Hz12,
        dlpf_enable: true,
        sample_rate_div: 24,
    };
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps1000,
        dlpf: GyroDlpf::Hz12,
        dlpf_enable: true,
        sample_rate_div: 24,
    };
    let mag_config = MagConfig {
        mode: icm20948::MagMode::Continuous10Hz,
    };

    let irq_config = InterruptConfig {
        raw_data_ready: true,
        ..Default::default()
    };
    if let Err(_) = imu.configure_accelerometer(accel_config) {
        defmt::error!("Error configuring accel");
    }
    if let Err(_) = imu.configure_gyroscope(gyro_config) {
        defmt::error!("Error configuring gyro");
    }
    if let Err(_) = imu.configure_interrupts(&irq_config) {
        defmt::error!("Error configuring interrupts");
    }

    imu.init_magnetometer(mag_config, &mut Delay {}).ok();

    loop {
        let accel = imu.read_accelerometer().unwrap_or(icm20948::AccelDataG {
            x: f32::INFINITY,
            y: f32::INFINITY,
            z: f32::INFINITY,
        });
        let gyro = imu
            .read_gyroscope_radians()
            .unwrap_or(icm20948::GyroDataRps {
                x: f32::INFINITY,
                y: f32::INFINITY,
                z: f32::INFINITY,
            });
        let mag = imu.read_magnetometer().unwrap_or(icm20948::MagDataUT {
            x: f32::INFINITY,
            y: f32::INFINITY,
            z: f32::INFINITY,
        });

        defmt::info!("Accel: {} {} {}", accel.x, accel.y, accel.z);
        defmt::info!("Gyro: {} {} {}", gyro.x, gyro.y, gyro.z);
        defmt::info!("Mag: {} {} {}", mag.x, mag.y, mag.z);

        store_icm20948_values(gyro, accel, mag).unwrap();
        lilos::time::sleep_for(Duration::from_millis(200)).await;
    }
}
