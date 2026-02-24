use core::{fmt::Write as _, hash::Hasher as _};

use hash32::FnvHasher;
use heapless::String;
use stm32_metapac as pac;

/// Compute a 32-bit unique serial number from the UID register value.
pub fn get_serial() -> u32 {
    let mut ctx: FnvHasher = Default::default();
    ctx.write(&pac::UID.uid(0).read().to_le_bytes());
    ctx.write(&pac::UID.uid(1).read().to_le_bytes());
    ctx.write(&pac::UID.uid(2).read().to_le_bytes());
    ctx.finish() as u32
}

/// Convert the 32-bit serial number to an 8-character uppercase hex string.
pub fn get_serial_str() -> String<8> {
    let serial = get_serial();
    let mut out = String::<8>::new();
    let _ = write!(&mut out, "{serial:08X}");
    out
}
