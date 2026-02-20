use core::convert::Infallible;

use rjmp_stm32_flash::{
    DualPageFlash, SectionUpdate, UpdateSource, load_sections, update_sections,
};
use zencan_node::common::NodeId;

/// Wrapper for embedded_io::Read to convert between embedded_io 0.6 and 0.7, used by zencan and
/// flash driver ugh
struct Read7<'a>(&'a mut dyn embedded_io::Read<Error = Infallible>);

impl rjmp_stm32_flash::embedded_io::ErrorType for Read7<'_> {
    type Error = Infallible;
}
impl rjmp_stm32_flash::embedded_io::Read for Read7<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.0.read(buf)
    }
}

#[repr(u8)]
pub enum FlashSections {
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

pub fn store_objects<E>(
    flash: &mut dyn DualPageFlash<Error = E>,
    reader: &mut dyn embedded_io::Read<Error = Infallible>,
    size: usize,
) {
    let mut read7 = Read7(reader);
    if update_sections(
        flash,
        &mut [SectionUpdate {
            section_id: FlashSections::Objects as u8,
            data: UpdateSource::Reader((&mut read7, size)),
        }],
    )
    .is_err()
    {
        defmt::error!("Error storing objects to flash");
    }
}

pub fn store_node_config<E>(flash: &mut dyn DualPageFlash<Error = E>, id: NodeId) {
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

pub fn read_persisted_objects<E>(
    flash: &mut dyn DualPageFlash<Error = E>,
    restore_fn: impl Fn(&[u8]),
) {
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
