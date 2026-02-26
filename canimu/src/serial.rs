use core::{fmt::Write as _, hash::Hasher as _};

use hash32::FnvHasher;
use heapless::String;
use lazy_static::lazy_static;
use stm32_metapac as pac;

/// Compute a 32-bit unique serial number from the UID register value.
pub fn get_serial() -> u32 {
    let mut ctx: FnvHasher = Default::default();
    ctx.write(&pac::UID.uid(0).read().to_le_bytes());
    ctx.write(&pac::UID.uid(1).read().to_le_bytes());
    ctx.write(&pac::UID.uid(2).read().to_le_bytes());
    ctx.finish() as u32
}

lazy_static! {
    static ref SERIAL_STR: String<8> = {
        let serial = get_serial();
        let mut out = String::<8>::new();
        let _ = write!(&mut out, "{serial:08X}");
        out
    };
}

/// Convert the 32-bit serial number to an 8-character uppercase hex string.
pub fn get_serial_str() -> &'static str {
    &SERIAL_STR
}
