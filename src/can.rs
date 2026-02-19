use core::{cell::RefCell, num::NonZeroU8};

use crate::pac;
use critical_section::Mutex;
use fdcan::{
    FdCan, NormalOperationMode,
    config::{DataBitTiming, FdCanConfig, GlobalFilter},
    filter::{StandardFilter, StandardFilterSlot},
};
use stm32_hal2::pac::interrupt;

struct FdCan1 {}

unsafe impl fdcan::message_ram::Instance for FdCan1 {
    const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = pac::FDCANRAM1.as_ptr() as _;
}
unsafe impl fdcan::Instance for FdCan1 {
    const REGISTERS: *mut fdcan::RegisterBlock = pac::FDCAN1.as_ptr() as _;
}

static CAN: Mutex<RefCell<Option<FdCan<FdCan1, NormalOperationMode>>>> =
    Mutex::new(RefCell::new(None));

fn zencan_to_fdcan_header(msg: &zencan_node::common::CanMessage) -> fdcan::frame::TxFrameHeader {
    let id: fdcan::id::Id = match msg.id() {
        zencan_node::common::messages::CanId::Extended(id) => {
            fdcan::id::ExtendedId::new(id).unwrap().into()
        }
        zencan_node::common::messages::CanId::Std(id) => {
            fdcan::id::StandardId::new(id).unwrap().into()
        }
    };
    fdcan::frame::TxFrameHeader {
        len: msg.dlc,
        frame_format: fdcan::frame::FrameFormat::Standard,
        id,
        bit_rate_switching: false,
        marker: None,
    }
}

/// Move outgoing CAN messages from NODE_MBOX to the CAN controller
///
/// Will move messages until either the hardware FIFO is full, or NODE_MBOX is out of messages.
fn transmit_can_messages(can: &mut FdCan<FdCan1, NormalOperationMode>) {
    loop {
        // Check if queue is full
        // Driver lacks API for this so go straight to register
        if pac::FDCAN1.txfqs().read().tfqf() {
            break;
        }
        if let Some(msg) = crate::zencan::NODE_MBOX.next_transmit_message() {
            let header = zencan_to_fdcan_header(&msg);
            if let Err(_) = can.transmit_preserve(header, msg.data(), &mut |_, _, _| {
                defmt::info!("Cancelled transmission");
            }) {
                defmt::error!("Error transmitting CAN message");
            }
        } else {
            break;
        }
    }
}

/// A handler for the zencan transmit notify callback. It accesses the CAN peripheral in a critical
/// section so it can be called from any context
pub fn transmit_notify_handler() {
    critical_section::with(|cs| {
        let mut borrow = CAN.borrow_ref_mut(cs);
        let can = borrow.as_mut().unwrap();
        transmit_can_messages(can);
    })
}

pub fn init_can() {
    let mut can = FdCan::new(FdCan1 {}).into_config_mode();
    // Bit timing calculated at http://www.bittiming.can-wiki.info/
    // 250kbit with 170MHz input clock (APB1)
    const PRESC: NonZeroU8 = NonZeroU8::new(40).unwrap();
    const SEG1: NonZeroU8 = NonZeroU8::new(14).unwrap();
    const SEG2: NonZeroU8 = NonZeroU8::new(2).unwrap();
    let can_config = FdCanConfig::default()
        .set_automatic_retransmit(false)
        .set_frame_transmit(fdcan::config::FrameTransmissionConfig::ClassicCanOnly)
        .set_data_bit_timing(DataBitTiming {
            transceiver_delay_compensation: false,
            prescaler: PRESC,
            seg1: SEG1,
            seg2: SEG2,
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        })
        .set_nominal_bit_timing(fdcan::config::NominalBitTiming {
            prescaler: PRESC.into(),
            seg1: SEG1,
            seg2: SEG2,
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        })
        .set_global_filter(GlobalFilter {
            handle_standard_frames: fdcan::config::NonMatchingFilter::IntoRxFifo0,
            handle_extended_frames: fdcan::config::NonMatchingFilter::IntoRxFifo0,
            reject_remote_standard_frames: false,
            reject_remote_extended_frames: false,
        });

    can.apply_config(can_config);
    let mut can = can.into_normal();

    // Set the per-mailbox TX interrupt to enable TXComplete IRQ
    // Works around a bug in fdcan driver: see https://github.com/stm32-rs/fdcan/issues/42
    pac::FDCAN1.txbtie().write(|w| w.0 = 7);
    can.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
    can.enable_interrupt(fdcan::interrupt::Interrupt::TxComplete);
    // Bug in FDCAN driver swaps interrupt line config, so configure _1 even though we want _0
    can.enable_interrupt_line(fdcan::config::InterruptLine::_1, true);
    can.set_standard_filter(
        StandardFilterSlot::_0,
        StandardFilter::accept_all_into_fifo0(),
    );

    // Store the CAN periph statically for the IRQ handler
    critical_section::with(|cs| {
        CAN.borrow_ref_mut(cs).replace(can);
    });
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::FDCAN1_IT0) };
}

// The CAN interrupt moves messages between the FDCAN peripheral and the node mailbox
//
// When new messages are queued, it is also required to push the first messages to the peripheral
// in the process thread. If further messages are queued to be sent, the tx complete interrupt will
// queue them in the background.
#[interrupt]
fn FDCAN1_IT0() {
    // Safety: No other IRQs access CAN, so no critical section is required in the IRQ
    let cs = unsafe { critical_section::CriticalSection::new() };

    let mut cell = CAN.borrow_ref_mut(cs);
    let can = cell.as_mut().unwrap();

    if can.has_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg) {
        can.clear_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
        let mut buffer = [0u8; 8];

        while let Ok(msg) = can.receive0(&mut buffer) {
            // ReceiveOverrun::unwrap() cannot fail
            let msg = msg.unwrap();

            let id = match msg.id {
                fdcan::id::Id::Standard(standard_id) => {
                    zencan_node::common::messages::CanId::std(standard_id.as_raw())
                }
                fdcan::id::Id::Extended(extended_id) => {
                    zencan_node::common::messages::CanId::extended(extended_id.as_raw())
                }
            };
            let msg =
                zencan_node::common::messages::CanMessage::new(id, &buffer[..msg.len as usize]);
            // Ignore error -- as an Err is returned for messages that are not consumed by the node
            // stack
            crate::zencan::NODE_MBOX.store_message(msg).ok();
        }
    }

    if can.has_interrupt(fdcan::interrupt::Interrupt::TxComplete) {
        can.clear_interrupt(fdcan::interrupt::Interrupt::TxComplete);
        transmit_can_messages(can);
    }
}
