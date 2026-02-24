use core::{convert::Infallible, mem::MaybeUninit, time::Duration};

use stm32_hal2::{
    self as hal,
    usb::{Peripheral, UsbBus},
};

use usb_device::prelude::*;
use usbd_dfu_rt::{DfuRuntimeClass, DfuRuntimeOps};

struct DfuOps {}

/// Just a random magic number
const MAGIC_JUMP_BOOTLOADER: u32 = 0x1081abcd;
/// Location of the system memory containing the bootloader
const SYSTEM_MEMORY_BASE: u32 = 0x1fff0000;

impl DfuRuntimeOps for DfuOps {
    #[allow(static_mut_refs)]
    fn detach(&mut self) {
        defmt::error!("DETACH");
        // unsafe {
        //     jump_bootloader();
        // }
        unsafe {
            MAGIC.as_mut_ptr().write(MAGIC_JUMP_BOOTLOADER);
        }
        cortex_m::peripheral::SCB::sys_reset();
    }
}
#[unsafe(link_section = ".uninit.MAGIC")]
static mut MAGIC: MaybeUninit<u32> = MaybeUninit::uninit();

#[allow(static_mut_refs)]
#[cortex_m_rt::pre_init]
unsafe fn jump_bootloader() {
    unsafe {
        if MAGIC.assume_init() == MAGIC_JUMP_BOOTLOADER {
            // reset the magic value not to jump again
            MAGIC.as_mut_ptr().write(0);
            // jump to bootloader located in System Memory
            cortex_m::asm::bootload(SYSTEM_MEMORY_BASE as *const u32);
        }
    }
}

pub async fn usb_task() -> Infallible {
    let dp = unsafe { hal::pac::Peripherals::steal() };
    let usb = Peripheral { regs: dp.USB };
    let usb_bus = UsbBus::new(usb);
    let mut dfu = DfuRuntimeClass::new(&usb_bus, DfuOps {});

    let serial = crate::serial::get_serial_str();
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xF0F0, 0x12ee))
        .strings(&[StringDescriptors::default()
            .manufacturer("Rising Tide Research Foundation")
            .product("CAN IMU")
            .serial_number(&serial)])
        .unwrap()
        .device_class(0xFF)
        .build();

    let mut last_time = lilos::time::TickTime::now();
    loop {
        usb_dev.poll(&mut [&mut dfu]);
        dfu.tick(last_time.elapsed().0 as u16);
        last_time = lilos::time::TickTime::now();
        lilos::time::sleep_for(Duration::from_millis(1)).await;
    }
}
