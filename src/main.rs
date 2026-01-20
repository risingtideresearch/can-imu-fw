#![no_std]
#![no_main]

use cortex_m_rt::{self as _};
use defmt::info;
use embedded_hal::{delay::DelayNs, spi::SpiBus as _};
use panic_probe as _;
use rtt_target::{rtt_init, set_defmt_channel};
use zencan_node::{Callbacks, Node, common::NodeId, object_dict::ObjectAccess};

use core::{convert::Infallible, f32, hash::Hasher as _, pin::pin, time::Duration};

use hash32::FnvHasher;
use lilos::{
    exec::{Interrupts, Notify},
    time::Millis,
};
use stm32_hal2::{
    self as hal,
    gpio::{OutputType, Pin, PinMode, Port},
    spi::{Spi, SpiConfig},
    usart::UsartConfig,
};

use icm20948::{MagConfig, SpiInterface};
use icm20948::{
    interface::Interface,
    sensors::{AccelConfig, GyroConfig},
};

use stm32_metapac as pac;

mod can;

mod zencan {
    zencan_node::include_modules!(ZENCAN_CONFIG);
}

// Placeholder until I sort a better driver -- wrap the stm32-hal2 driver to give it a SpiBus impl
struct SpiBus(Spi<hal::pac::SPI1>);

#[derive(Debug)]
pub struct SpiError {}
impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}
impl embedded_hal::spi::ErrorType for SpiBus {
    type Error = SpiError;
}

impl embedded_hal::spi::SpiBus for SpiBus {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.0.transfer(words).map_err(|_| SpiError {})
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.0
            .transfer_type2(words, &mut [])
            .map_err(|_| SpiError {})
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.0.transfer_type2(write, read).map_err(|_| SpiError {})
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.0.transfer(words).map_err(|_| SpiError {})
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

static CAN_NOTIFY: Notify = Notify::new();

/// Callback to notify CAN task that there are messages to be processed
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

struct Delay {}

impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        let start = lilos::time::TickTime::now();
        while start.elapsed().0 * 1000000 < ns as u64 {
            defmt::info!("Waiting {} {}", start.elapsed().0, ns);
        }
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
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

    let clock_cfg = hal::clocks::Clocks::default();
    clock_cfg.setup().unwrap();

    info!("APB1: {}", clock_cfg.apb1());
    let dp = hal::pac::Peripherals::take().unwrap();

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

    info!("Setting up UART1");

    let uart =
        hal::usart::Usart::new(dp.USART1, 115200, UsartConfig::default(), &clock_cfg).unwrap();

    // let spi = hal::spi::Spi::new(
    //     dp.SPI1,
    //     SpiConfig::default(),
    //     stm32_hal2::spi::BaudRate::Div32,
    // );

    info!("Setting up NODE");
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
        clock_div: rjmp_stm32_spi::ClockDiv::Div256,
        bit_order: rjmp_stm32_spi::BitOrder::MsbFirst,
    };
    defmt::info!("Creating SPI");
    let mut spi = rjmp_stm32_spi::Spi::spi1(spi_config);
    let device = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, icm_cs).unwrap();
    defmt::info!("Creating IMU");
    let mut imu = icm20948::Icm20948Driver::new(SpiInterface::new(device)).unwrap();
    defmt::info!("Init IMU");
    imu.init(&mut Delay {}).unwrap();
    defmt::info!("Starting tasks");
    unsafe {
        lilos::exec::run_tasks_with_preemption(
            &mut [pin!(can_task(node)), pin!(imu_task(imu))],
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

fn read_spi(spi: &mut SpiBus, addr: u8, buf: &mut [u8]) -> Result<(), SpiError> {
    // spi.transfer_in_place(&mut [addr | (1 << 7)])?;
    // spi.transfer_in_place(buf)?;

    spi.write(&[addr | 1 << 7])?;
    spi.read(buf)?;
    Ok(())
}

async fn imu_task<I: Interface>(mut imu: icm20948::Icm20948Driver<I>) -> Infallible {
    use icm20948::{AccelDlpf, AccelFullScale, GyroDlpf, GyroFullScale, InterruptConfig};

    let mut whoami = [0];

    lilos::time::sleep_for(Duration::from_millis(100)).await;

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

        const GYRO_RANGE: f32 = 1000.0 * core::f32::consts::PI / 180.0;
        zencan::OBJECT2000
            .set(0, (gyro.x * 32768.0 / GYRO_RANGE) as i16)
            .unwrap();
        zencan::OBJECT2000
            .set(1, (gyro.y * 32768.0 / GYRO_RANGE) as i16)
            .unwrap();
        zencan::OBJECT2000
            .set(2, (gyro.z * 32768.0 / GYRO_RANGE) as i16)
            .unwrap();

        const ACCELL_RANGE: f32 = 8.0;
        zencan::OBJECT2001
            .set(0, (accel.x * 32768.0 / ACCELL_RANGE) as i16)
            .unwrap();
        zencan::OBJECT2001
            .set(1, (accel.y * 32768.0 / ACCELL_RANGE) as i16)
            .unwrap();
        zencan::OBJECT2001
            .set(2, (accel.z * 32768.0 / ACCELL_RANGE) as i16)
            .unwrap();

        zencan::OBJECT2000.set_event_flag(1).unwrap();
        zencan::OBJECT2001.set_event_flag(2).unwrap();

        lilos::time::sleep_for(Duration::from_millis(200)).await;
    }
}
