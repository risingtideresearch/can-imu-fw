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
    Callbacks, Node,
    common::{NodeId, sdo::AbortCode},
    object_dict::{ODEntry, ObjectAccess},
    restore_stored_comm_objects, restore_stored_objects,
};

use num_traits::float::Float;

use core::{cell::RefCell, convert::Infallible, f32, hash::Hasher as _, pin::pin, time::Duration};

use hash32::FnvHasher;
use lilos::exec::{Interrupts, Notify};
use stm32_hal2::{
    self as hal,
    gpio::{Edge, Pin, PinMode, Port},
    pac::interrupt,
    usart::UsartConfig,
};

use icm20948::{
    AccelDlpf, AccelFullScale, GyroDlpf, GyroFullScale, InterruptConfig, MagConfig, SpiInterface,
    sensors::{AccelConfig, GyroConfig},
};

use stm32_metapac as pac;

use rjmp_stm32_flash::{
    DualPageFlash, SectionUpdate, Stm32gxFlash, Stm32gxPagePair, UpdateSource, load_sections,
    update_sections, write_section,
};

/// can module wires up CAN message sending and receiving
mod can;
/// state module provides state estimation algorithms
mod state;

// Instantiate zencan static objects
mod zencan {
    zencan_node::include_modules!(ZENCAN_CONFIG);
}

/// A notification for the ICM20948 IRQ
static ICM_NOTIFY: Notify = Notify::new();

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

#[repr(u8)]
enum FlashSections {
    NodeConfig = 1,
    Objects = 2,
    Unknown = 255,
}

impl From<u8> for FlashSections {
    fn from(value: u8) -> Self {
        match value {
            1 => Self::NodeConfig,
            2 => Self::Objects,
            _ => Self::Unknown,
        }
    }
}

fn store_objects<E>(
    flash: &mut dyn DualPageFlash<Error = E>,
    reader: &mut dyn embedded_io::Read<Error = Infallible>,
    size: usize,
) {
    if update_sections(
        flash,
        &mut [SectionUpdate {
            section_id: FlashSections::Objects as u8,
            data: UpdateSource::Reader((reader, size)),
        }],
    )
    .is_err()
    {
        defmt::error!("Error storing objects to flash");
    }
}

fn store_node_config<E>(flash: &mut dyn DualPageFlash<Error = E>, id: NodeId) {
    //let mut flash = unsafe { FLASH.as_mut().unwrap().unlock() };
    let data = [id.raw()];
    if update_sections(
        flash,
        &mut [SectionUpdate {
            section_id: FlashSections::NodeConfig as u8,
            data: UpdateSource::Slice(&data),
        }],
    )
    .is_err()
    {
        defmt::error!("Error storing node config to flash");
    }
}

fn read_persisted_objects<E>(flash: &mut dyn DualPageFlash<Error = E>, restore_fn: impl Fn(&[u8])) {
    if let Some(sections) = load_sections(flash) {
        for s in sections {
            let section_type = FlashSections::from(s.section_id);
            match section_type {
                FlashSections::NodeConfig => (), // Ignore
                FlashSections::Objects => {
                    defmt::info!("Loaded objects from flash");
                    restore_fn(s.data);
                }
                FlashSections::Unknown => {
                    defmt::warn!("Found unrecognized flash section {}", s.section_id);
                }
            }
        }
    } else {
        defmt::info!("No data found in flash");
    }
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

    // Setup ICM IMU pins on SPI1
    let _icm_sdi = Pin::new(Port::A, 7, PinMode::Alt(5));
    let _icm_sdo = Pin::new(Port::A, 6, PinMode::Alt(5));
    let _icm_sck = Pin::new(Port::A, 5, PinMode::Alt(5));
    let mut icm_cs = Pin::new(Port::B, 0, PinMode::Output);
    icm_cs.set_high();

    let mut icm_int1 = Pin::new(Port::B, 1, PinMode::Input);
    icm_int1.enable_interrupt(Edge::Rising);

    let _uart =
        hal::usart::Usart::new(dp.USART1, 115200, UsartConfig::default(), &clock_cfg).unwrap();

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

fn gyro_rps_from_raw(gyro: (i16, i16, i16)) -> (f32, f32, f32) {
    const DEG2RAD: f32 = core::f32::consts::PI / 180.0;
    const GYRO_SCALE: f32 = 500.0 * DEG2RAD / 32768.0;
    (
        gyro.0 as f32 * GYRO_SCALE,
        gyro.1 as f32 * GYRO_SCALE,
        gyro.2 as f32 * GYRO_SCALE,
    )
}

const G_MPSS: f32 = 9.80665;

fn accel_mpss_from_raw(accel: (i16, i16, i16)) -> (f32, f32, f32) {
    const ACCEL_SCALE: f32 = 4.0 * G_MPSS / 32768.0; // m/s/s per LSB
    (
        accel.0 as f32 * ACCEL_SCALE,
        accel.1 as f32 * ACCEL_SCALE,
        accel.2 as f32 * ACCEL_SCALE,
    )
}

fn accel_g_from_raw(accel: (i16, i16, i16)) -> (f32, f32, f32) {
    const ACCEL_SCALE: f32 = 4.0 / 32768.0; // g per LSB
    (
        accel.0 as f32 * ACCEL_SCALE,
        accel.1 as f32 * ACCEL_SCALE,
        accel.2 as f32 * ACCEL_SCALE,
    )
}

fn offset_gyros(mut gyro: (i16, i16, i16)) -> (i16, i16, i16) {
    let gyro_x_offset = zencan::OBJECT2010.get(0).unwrap_or(0);
    let gyro_y_offset = zencan::OBJECT2010.get(1).unwrap_or(0);
    let gyro_z_offset = zencan::OBJECT2010.get(2).unwrap_or(0);
    gyro.0 = gyro.0.saturating_sub(gyro_x_offset);
    gyro.1 = gyro.1.saturating_sub(gyro_y_offset);
    gyro.2 = gyro.2.saturating_sub(gyro_z_offset);
    gyro
}

fn offset_accels(accel: (i16, i16, i16)) -> (i16, i16, i16) {
    let x_off = zencan::OBJECT2020.get(0).unwrap_or(0);
    let y_off = zencan::OBJECT2020.get(1).unwrap_or(0);
    let z_off = zencan::OBJECT2020.get(2).unwrap_or(0);
    (
        accel.0.saturating_sub(x_off),
        accel.1.saturating_sub(y_off),
        accel.2.saturating_sub(z_off),
    )
}

/// Rotate sensor readings into the desired coordinate frame.
fn rotate_sensors(raw: (i16, i16, i16)) -> (i16, i16, i16) {
    (-raw.1, -raw.0, -raw.2)
}

#[inline]
/// Function to store IMU data to object dict
///
/// Mainly motivated by the belief that it will create less panic sites and therefore smaller code
/// although this is an untested theory
fn store_icm20948_values(
    gyro: (i16, i16, i16),
    accel: (i16, i16, i16),
    mag: (i16, i16, i16),
) -> Result<(), AbortCode> {
    const MAG_SCALE: f32 = 1.0 / 0.015; // LSB per uT

    let gyro_rps = gyro_rps_from_raw(gyro);
    let accel_g = accel_g_from_raw(accel);

    zencan::OBJECT2000.set(0, gyro.0)?;
    zencan::OBJECT2000.set(1, gyro.1)?;
    zencan::OBJECT2000.set(2, gyro.2)?;
    zencan::OBJECT2001.set(0, gyro_rps.0)?;
    zencan::OBJECT2001.set(1, gyro_rps.1)?;
    zencan::OBJECT2001.set(2, gyro_rps.2)?;

    zencan::OBJECT2002.set(0, accel.0)?;
    zencan::OBJECT2002.set(1, accel.1)?;
    zencan::OBJECT2002.set(2, accel.2)?;
    zencan::OBJECT2003.set(0, accel_g.0)?;
    zencan::OBJECT2003.set(1, accel_g.1)?;
    zencan::OBJECT2003.set(2, accel_g.2)?;

    zencan::OBJECT2004.set(0, mag.0)?;
    zencan::OBJECT2004.set(1, mag.1)?;
    zencan::OBJECT2004.set(2, mag.2)?;
    zencan::OBJECT2005.set(0, mag.0 as f32 * MAG_SCALE)?;
    zencan::OBJECT2005.set(1, mag.1 as f32 * MAG_SCALE)?;
    zencan::OBJECT2005.set(2, mag.2 as f32 * MAG_SCALE)?;

    for i in 0..3 {
        zencan::OBJECT2000.set_event_flag(i)?;
        zencan::OBJECT2001.set_event_flag(i)?;
        zencan::OBJECT2002.set_event_flag(i)?;
        zencan::OBJECT2003.set_event_flag(i)?;
        zencan::OBJECT2004.set_event_flag(i)?;
        zencan::OBJECT2005.set_event_flag(i)?;
    }

    // Notify process task it needs to run
    notify_can_task();
    Ok(())
}

/// A task for reading data from the IMU
async fn imu_task<S: embedded_hal::spi::SpiBus>(spi: S, cs: Pin) -> Infallible {
    /// The number of samples to collect during gyro calibration
    const GYRO_CAL_SAMPLES: usize = 200;
    const OUTPUT_PERIOD: f32 = 0.1;
    const FSDIV: u16 = 11;
    let device = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let mut imu = icm20948::Icm20948Driver::new(SpiInterface::new(device)).unwrap();

    let mut cal_sample_counter: Option<usize> = None;
    let mut gyro_accum = [0i32, 0, 0];
    let sample_period = FSDIV as f32 / 1125.0;

    let mut filter = state::Filter::new(sample_period, OUTPUT_PERIOD);

    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G4,
        dlpf: AccelDlpf::Hz12,
        dlpf_enable: true,
        sample_rate_div: FSDIV - 1,
    };
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps500,
        dlpf: GyroDlpf::Hz12,
        dlpf_enable: true,
        sample_rate_div: FSDIV as u8 - 1,
    };
    let mag_config = MagConfig {
        mode: icm20948::MagMode::Continuous10Hz,
    };

    let irq_config = InterruptConfig {
        raw_data_ready: true,
        ..Default::default()
    };

    if let Err(_) = imu.init(&mut Delay {}) {
        defmt::error!("Error initializing ICM");
    }
    if let Err(_) = imu.init_magnetometer(mag_config, &mut Delay {}) {
        defmt::error!("Error from init_magnetometer");
    }
    if let Err(_) = imu.configure_accelerometer(accel_config) {
        defmt::error!("Error configuring accel");
    }
    if let Err(_) = imu.configure_gyroscope(gyro_config) {
        defmt::error!("Error configuring gyro");
    }
    if let Err(_) = imu.configure_interrupts(&irq_config) {
        defmt::error!("Error configuring interrupts");
    }

    loop {
        // Wait for a notification
        lilos::time::with_timeout(Duration::from_millis(100), ICM_NOTIFY.until_next()).await;
        // Check status flag, so see if new data is actually available
        let flag = imu
            .device_mut()
            .int_status_1()
            .read()
            .unwrap()
            .raw_data_0_rdy_int();
        if !flag {
            info!("NO FLAG");
            continue;
        }

        let accel = rotate_sensors(imu.read_accelerometer_raw().unwrap_or((
            i16::MAX,
            i16::MAX,
            i16::MAX,
        )));
        let gyro = rotate_sensors(imu.read_gyroscope_raw().unwrap_or((
            i16::MAX,
            i16::MAX,
            i16::MAX,
        )));
        let mag = imu
            .read_magnetometer_raw()
            .unwrap_or((i16::MAX, i16::MAX, i16::MAX));

        // Check if calibration is in progress or requested
        let calibration_control = zencan::OBJECT2011.get_value();

        if calibration_control & (1 << 0) != 0 {
            cal_sample_counter = Some(0);
            gyro_accum = [0, 0, 0];
            // Set calibration in progress bit
            zencan::OBJECT2011.set_value(1 << 1);
        }

        if cal_sample_counter.is_some() {
            // In progress cal
            gyro_accum[0] += gyro.0 as i32;
            gyro_accum[1] += gyro.1 as i32;
            gyro_accum[2] += gyro.2 as i32;
            *cal_sample_counter.as_mut().unwrap() += 1;

            if cal_sample_counter.unwrap() == GYRO_CAL_SAMPLES {
                for i in 0..3 {
                    zencan::OBJECT2010
                        .set(i, (gyro_accum[i] / GYRO_CAL_SAMPLES as i32) as i16)
                        .unwrap();
                }
                cal_sample_counter = None;
                zencan::OBJECT2011.set_value(0);
            }
        }

        let gyro = offset_gyros(gyro);
        let accel = offset_accels(accel);
        store_icm20948_values(gyro, accel, mag).unwrap();

        // Do state estimate update
        let gyro_rps = gyro_rps_from_raw(gyro);
        let accel_mpss = accel_mpss_from_raw(accel);

        if let Some(state::FilterSample { pitch, roll, heave }) =
            filter.push_sample(accel_mpss, gyro_rps)
        {
            // Set float tilt
            zencan::OBJECT2111.set_pitch(pitch);
            zencan::OBJECT2111.set_roll(roll);
            // Set integer tilt
            zencan::OBJECT2110.set_pitch((pitch * 1000.0).round() as i16);
            zencan::OBJECT2110.set_roll((roll * 1000.0).round() as i16);
            // Set float heave
            zencan::OBJECT2101.set_value(heave);
            // Set integer heave
            zencan::OBJECT2100.set_value((heave * 100.0).round() as i16);

            zencan::OBJECT2110.set_event_flag(1).ok();
            zencan::OBJECT2110.set_event_flag(2).ok();
            zencan::OBJECT2111.set_event_flag(1).ok();
            zencan::OBJECT2111.set_event_flag(2).ok();
            zencan::OBJECT2100.set_event_flag(0).ok();
            zencan::OBJECT2101.set_event_flag(0).ok();
            notify_can_task();
        }
    }
}

#[interrupt]
fn EXTI1() {
    hal::gpio::clear_exti_interrupt(1);
    ICM_NOTIFY.notify();
}
