//! UUID serial number helpers
//!
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

        fn nibble_to_char(x: u8) -> char {
            match x {
                0 => '0',
                1 => '1',
                2 => '2',
                3 => '3',
                4 => '4',
                5 => '5',
                6 => '6',
                7 => '7',
                8 => '8',
                9 => '9',
                10 => 'A',
                11 => 'B',
                12 => 'C',
                13 => 'D',
                14 => 'E',
                15 => 'F',
                _ => '_',
            }
        }
        let _ = out.write_char(nibble_to_char(((serial >> 0) & 0x0F) as u8));
        let _ = out.write_char(nibble_to_char(((serial >> 4) & 0x0F) as u8));
        let _ = out.write_char(nibble_to_char(((serial >> 8) & 0x0F) as u8));
        let _ = out.write_char(nibble_to_char(((serial >> 12) & 0x0F) as u8));
        let _ = out.write_char(nibble_to_char(((serial >> 16) & 0x0F) as u8));
        let _ = out.write_char(nibble_to_char(((serial >> 20) & 0x0F) as u8));
        let _ = out.write_char(nibble_to_char(((serial >> 24) & 0x0F) as u8));
        let _ = out.write_char(nibble_to_char(((serial >> 28) & 0x0F) as u8));

        out
    };
}

/// Convert the 32-bit serial number to an 8-character uppercase hex string.
pub fn get_serial_str() -> &'static str {
    &SERIAL_STR
}
