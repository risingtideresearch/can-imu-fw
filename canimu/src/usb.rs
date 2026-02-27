use core::{
    cell::UnsafeCell,
    convert::Infallible,
    mem::MaybeUninit,
    sync::atomic::{AtomicBool, Ordering},
    time::Duration,
};

use embedded_can::{ExtendedId, Frame as _, Id, StandardId};
use stm32_hal2::{
    self as hal,
    usb::{Peripheral, UsbBus},
};

use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_dfu_rt::{DfuRuntimeClass, DfuRuntimeOps};
use usbd_gscan::{
    Device, GsCan,
    host::{
        CanBitTimingConst, CanState, DeviceBitTiming, DeviceBitTimingConst,
        DeviceBitTimingConstExtended, DeviceConfig, DeviceState, Feature, FrameFlag,
    },
};
use zencan_node::common::{CanId, CanMessage};

/// Just a random magic number
const MAGIC_JUMP_BOOTLOADER: u32 = 0x1081abcd;
/// Location of the system memory containing the bootloader
const SYSTEM_MEMORY_BASE: u32 = 0x1fff0000;

pub static DFU_RESET_REQ: AtomicBool = AtomicBool::new(false);

struct DfuOps {}

impl DfuRuntimeOps for DfuOps {
    fn detach(&mut self) {
        defmt::info!("DETACH requested");
        DFU_RESET_REQ.store(true, Ordering::Relaxed);
    }
}

#[unsafe(link_section = ".uninit.MAGIC")]
static mut MAGIC: MaybeUninit<u32> = MaybeUninit::uninit();

#[allow(static_mut_refs)]
fn reset_to_bootloader() {
    unsafe {
        MAGIC.as_mut_ptr().write(MAGIC_JUMP_BOOTLOADER);
    }
    cortex_m::peripheral::SCB::sys_reset();
}

#[allow(static_mut_refs)]
#[cortex_m_rt::pre_init]
unsafe fn check_jump_bootloader() {
    unsafe {
        if MAGIC.assume_init() == MAGIC_JUMP_BOOTLOADER {
            // reset the magic value not to jump again
            MAGIC.as_mut_ptr().write(0);
            // jump to bootloader located in System Memory
            cortex_m::asm::bootload(SYSTEM_MEMORY_BASE as *const u32);
        }
    }
}

const TIMING_NOMINAL: CanBitTimingConst = CanBitTimingConst {
    tseg1_min: 1,
    tseg1_max: 255,
    tseg2_min: 1,
    tseg2_max: 127,
    sjw_max: 127,
    brp_min: 1,
    brp_max: 511,
    brp_inc: 1,
};
const TIMING_DATA: CanBitTimingConst = CanBitTimingConst {
    tseg1_min: 1,
    tseg1_max: 31,
    tseg2_min: 1,
    tseg2_max: 15,
    sjw_max: 15,
    brp_min: 1,
    brp_max: 31,
    brp_inc: 1,
};

static CAN_ACTIVE: AtomicBool = AtomicBool::new(false);
struct GsCanDevice {}

impl Device for GsCanDevice {
    fn config(&self) -> DeviceConfig {
        DeviceConfig::new(1)
    }

    fn bit_timing(&self) -> DeviceBitTimingConst {
        DeviceBitTimingConst {
            features: Feature::GET_STATE | Feature::BT_CONST_EXT,
            fclk_can: 170_000_000,
            timing: TIMING_NOMINAL,
        }
    }

    fn bit_timing_ext(&self) -> DeviceBitTimingConstExtended {
        DeviceBitTimingConstExtended {
            features: Feature::GET_STATE | Feature::BT_CONST_EXT,
            fclk_can: 170_000_000,
            timing_nominal: TIMING_NOMINAL,
            timing_data: TIMING_DATA,
        }
    }

    fn configure_bit_timing(&mut self, _interface: u8, _timing: DeviceBitTiming) {
        // NOOP - We don't actually support changing bit timing
    }

    fn configure_bit_timing_data(&mut self, _interface: u8, _timing: DeviceBitTiming) {}

    fn reset(&mut self, _interface: u8) {
        // Turn off message transmission
        CAN_ACTIVE.store(false, Ordering::Relaxed);
    }

    fn start(&mut self, _interface: u8, _features: Feature) {
        // Turn on message transmission
        CAN_ACTIVE.store(true, Ordering::Relaxed);
    }

    fn state(&self, _interface: u8) -> DeviceState {
        DeviceState {
            state: CanState::Active,
            rx_errors: 0,
            tx_errors: 0,
        }
    }

    fn receive(&mut self, _interface: u8, frame: &usbd_gscan::host::Frame) {
        let id = match frame.id() {
            Id::Standard(id) => CanId::std(id.as_raw()),
            Id::Extended(id) => CanId::extended(id.as_raw()),
        };

        let msg = if frame.is_remote_frame() {
            CanMessage::new_rtr(id)
        } else {
            CanMessage::new(id, frame.data())
        };

        crate::zencan::NODE_MBOX.store_message(msg).ok();
        crate::notify_can_task();
        crate::can::queue_usb_to_can(msg);
        crate::can::transmit_notify_handler();
    }
}

fn zencan_to_gscan_frame(msg: CanMessage) -> Option<usbd_gscan::host::Frame> {
    let id = match msg.id() {
        CanId::Std(id) => Id::Standard(StandardId::new(id)?),
        CanId::Extended(id) => Id::Extended(ExtendedId::new(id)?),
    };

    if msg.is_rtr() {
        <usbd_gscan::host::Frame as embedded_can::Frame>::new_remote(id, msg.dlc as usize)
    } else {
        <usbd_gscan::host::Frame as embedded_can::Frame>::new(id, msg.data())
    }
}

struct Global<T> {
    value: UnsafeCell<MaybeUninit<T>>,
    stored: AtomicBool,
}

impl<T> Global<T> {
    pub const fn empty() -> Self {
        Self {
            value: UnsafeCell::new(MaybeUninit::uninit()),
            stored: AtomicBool::new(false),
        }
    }

    /// Get the object using a critical section
    ///
    /// This does not check if the object has been initialized
    pub fn with<F: FnOnce(&mut T)>(&self, f: F) {
        if !self.stored.load(Ordering::Acquire) {
            panic!("Global not initialized");
        }
        critical_section::with(|_cs| unsafe {
            let value = &mut *self.value.get();
            f(value.assume_init_mut());
        });
    }

    pub fn init(&self, value: T) {
        unsafe {
            core::ptr::write(self.value.get(), MaybeUninit::new(value));
        }
        self.stored.store(true, Ordering::Release);
    }

    /// Only safe to call if you know that know other reference can exist
    pub unsafe fn steal(&self) -> &mut T {
        unsafe { (&mut *self.value.get()).assume_init_mut() }
    }
}

unsafe impl<T> Sync for Global<T> {}

static BUS_ALLOCATOR: Global<UsbBusAllocator<UsbBus<Peripheral>>> = Global::empty();
static USB_DEV: Global<UsbDevice<'static, UsbBus<Peripheral>>> = Global::empty();
static DFU: Global<DfuRuntimeClass<DfuOps>> = Global::empty();
static GSCAN: Global<GsCan<'static, UsbBus<Peripheral>, GsCanDevice>> = Global::empty();

pub async fn usb_task() -> Infallible {
    let dp = unsafe { hal::pac::Peripherals::steal() };
    let usb = Peripheral { regs: dp.USB };
    BUS_ALLOCATOR.init(UsbBus::new(usb));

    let gscan = GsCan::new(unsafe { BUS_ALLOCATOR.steal() }, GsCanDevice {});
    let dfu = DfuRuntimeClass::new(unsafe { BUS_ALLOCATOR.steal() }, DfuOps {});

    let serial = crate::serial::get_serial_str();
    //let vid_pid = identifier::CANDLELIGHT;
    let vid_pid = UsbVidPid(0x1209, 0x81fe);
    let usb_dev = UsbDeviceBuilder::new(unsafe { BUS_ALLOCATOR.steal() }, vid_pid)
        .strings(&[StringDescriptors::default()
            .manufacturer("Rising Tide Research Foundation")
            .product("CAN IMU")
            .serial_number(&serial)])
        .unwrap()
        .device_class(0xFF)
        .usb_rev(usb_device::device::UsbRev::Usb200)
        .device_sub_class(0xFF)
        .device_protocol(0xFF)
        .build();

    USB_DEV.init(usb_dev);
    GSCAN.init(gscan);
    DFU.init(dfu);

    unsafe { cortex_m::peripheral::NVIC::unmask(hal::pac::Interrupt::USB_LP) };
    let mut last_time = lilos::time::TickTime::now();
    loop {
        if DFU_RESET_REQ.load(Ordering::Relaxed) {
            //  Turn off the IMU, because the incoming data causes the system bootloader to go into
            //  UART mode instead of DFU
            defmt::info!("Resetting CM7");
            let f = crate::CM7_IMU
                .send_command_field(&microstrain_inertial::api::commands::base::ResetDevice {});

            crate::enable_uart_tx_irq();
            f.await.ok();
            cortex_m::interrupt::disable();
            defmt::info!("CM7 reset complete. Rebooting.");
            reset_to_bootloader();
        }
        while let Some(msg) = crate::can::dequeue_can_to_usb() {
            if !CAN_ACTIVE.load(Ordering::Relaxed) {
                continue;
            }

            if let Some(frame) = zencan_to_gscan_frame(msg) {
                GSCAN.with(|gscan| {
                    gscan.transmit(0, &frame, FrameFlag::empty());
                });
            }
        }
        let now = lilos::time::TickTime::now();
        let elapsed = now.millis_since(last_time).0;
        last_time = now;
        DFU.with(|dfu| {
            dfu.tick(elapsed as u16);
        });
        lilos::time::sleep_for(Duration::from_millis(1)).await;
    }
}

use stm32_hal2::pac::interrupt;
#[interrupt]
#[allow(static_mut_refs)]
fn USB_LP() {
    let usb_dev = unsafe { USB_DEV.steal() };
    let dfu = unsafe { DFU.steal() };
    let gscan = unsafe { GSCAN.steal() };

    usb_dev.poll(&mut [gscan, dfu]);
}
