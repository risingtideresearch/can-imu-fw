//! UART to USB bridge
#![no_std]
#![no_main]

use cortex_m_rt::{self as _};
use defmt::info;
use panic_probe as _;
use rtt_target::{ChannelMode, rtt_init, set_defmt_channel};

use stm32_hal2::{
    self as hal,
    gpio::{Pin, PinMode, Port},
    pac::{USART1, interrupt},
    usart::{Usart, UsartConfig, UsartInterrupt},
    usb::{Peripheral, UsbBus},
};

use usb_device::prelude::*;
use usbd_serial::SerialPort;

use stm32_metapac as pac;

static USB_BUFFER: bbqueue::nicknames::Churrasco<128> = bbqueue::nicknames::Churrasco::new();
static UART_BUFFER: bbqueue::nicknames::Churrasco<128> = bbqueue::nicknames::Churrasco::new();

#[allow(static_mut_refs)]
static mut UART: Option<Usart<USART1>> = None;

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

    defmt::error!("Setting up clocks");

    let dp = hal::pac::Peripherals::take().unwrap();
    let mut clock_cfg = hal::clocks::Clocks::default();
    clock_cfg.hsi48_on = true;
    clock_cfg.setup().unwrap();
    stm32_hal2::clocks::enable_crs(stm32_hal2::clocks::CrsSyncSrc::Usb);

    info!("Sysclk: {}", clock_cfg.sysclk());
    info!("APB1: {}", clock_cfg.apb1());
    info!("APB2: {}", clock_cfg.apb2());

    // Enable clocks
    pac::RCC.apb1enr1().modify(|w| w.set_usben(true));
    pac::RCC
        .ccipr()
        .modify(|w| w.set_clk48sel(stm32_metapac::rcc::vals::Clk48sel::HSI48));
    pac::RCC.apb1enr1().modify(|w| w.set_fdcanen(true));
    pac::RCC.apb1smenr1().modify(|w| w.set_fdcansmen(true));
    pac::RCC.apb1smenr1().modify(|w| w.set_crssmen(true));
    pac::RCC.apb1smenr1().modify(|w| w.set_usbsmen(true));
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

    // CAN Transceiver put into standby operating mode
    let mut can_stby = Pin::new(Port::B, 6, PinMode::Output);
    can_stby.set_high();
    let mut can_shdn = Pin::new(Port::B, 4, PinMode::Output);
    can_shdn.set_high();

    // setup 3DM uart pins for Uart1
    let _3dm_rx = Pin::new(Port::A, 10, PinMode::Alt(7));
    let _3dm_tx = Pin::new(Port::A, 9, PinMode::Alt(7));

    let mut uart =
        hal::usart::Usart::new(dp.USART1, 115200, UsartConfig::default(), &clock_cfg).unwrap();
    uart.enable().unwrap();
    uart.enable_interrupt(UsartInterrupt::TransmitEmpty)
        .unwrap();
    uart.enable_interrupt(UsartInterrupt::ReadNotEmpty).unwrap();

    unsafe { UART = Some(uart) };

    let usb = Peripheral { regs: dp.USB };
    let usb_bus = UsbBus::new(usb);

    let mut usb_serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Rising Tide Research Foundation")
            .product("Microstrain Serial Bridge")
            .serial_number("SN")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    unsafe { cortex_m::interrupt::enable() };
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART1) };

    defmt::info!("Starting tasks");

    let uart_consumer = UART_BUFFER.stream_consumer();
    let usb_producer = USB_BUFFER.stream_producer();
    let mut rx_buf = [0u8; 8];
    loop {
        usb_dev.poll(&mut [&mut usb_serial]);
        if let Ok(chunk) = uart_consumer.read() {
            defmt::info!("Got {} UART bytes", chunk.len());

            match usb_serial.write(&chunk) {
                Ok(0) | Err(UsbError::WouldBlock) => (),
                Ok(cnt) => {
                    chunk.release(cnt);
                }
                Err(_) => defmt::error!("USB error"),
            }

            while usb_serial.flush().err() == Some(UsbError::WouldBlock) {
                usb_dev.poll(&mut [&mut usb_serial]);
            }
        }

        if let Ok(count) = usb_serial.read(&mut rx_buf) {
            defmt::info!("Got {} USB bytes: {:?}", count, &rx_buf[..count]);
            if let Ok(mut grant) = usb_producer.grant_exact(count) {
                grant.copy_from_slice(&rx_buf[..count]);
                grant.commit(count);
                // enable the TXE IRQ
                pac::USART1.cr1().modify(|w| w.set_txeie(true));
            } else {
                defmt::error!("USB rx overrun");
            }
        }
    }
}
#[allow(static_mut_refs)]
#[interrupt]
fn USART1() {
    let uart_producer = UART_BUFFER.stream_producer();
    let usb_consumer = USB_BUFFER.stream_consumer();
    let uart = unsafe { UART.as_mut().unwrap() };

    if uart.check_status_flag(stm32_hal2::usart::UsartInterrupt::ReadNotEmpty) {
        let byte = uart.read_one();
        if let Ok(mut grant) = uart_producer.grant_exact(1) {
            grant[0] = byte;
            grant.commit(1);
        } else {
            defmt::error!("UART overrun");
        }
    }
    if uart.check_status_flag(stm32_hal2::usart::UsartInterrupt::TransmitEmpty) {
        if let Ok(data) = usb_consumer.read() {
            uart.write_one(data[0]);
            data.release(1);
        } else {
            uart.disable_interrupt(UsartInterrupt::TransmitEmpty);
        }
    }
}
