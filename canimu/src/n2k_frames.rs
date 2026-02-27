//! NMEA2000 CAN definitions
//!

#![allow(dead_code)]
use zencan_node::common::CanId;

pub struct HeaveFrame {
    /// Sequence ID for the measurement
    ///
    /// This can be used to indicate which data are co-samples, but on the Furuno SCX-20 this is
    /// always 0xFF/unused.
    pub sid: u8,
    /// Vertical displacement from mean sea level in meters, positive down
    pub heave: f32,
}

impl HeaveFrame {
    pub const PGN: u32 = 127252;
    const SCALE: f32 = 0.01;

    pub fn encode(&self, data: &mut [u8]) {
        let heave = (self.heave / Self::SCALE) as i16;
        data[0] = self.sid;
        data[1..3].copy_from_slice(&heave.to_le_bytes());
    }
}

pub struct AttitudeFrame {
    /// Sequence ID
    pub sid: u8,
    /// Yaw euler angle in rad
    pub yaw: Option<f32>,
    /// Pitch euler angle in rad
    pub pitch: Option<f32>,
    /// Roll euler angle in rad
    pub roll: Option<f32>,
}

fn wrap_rad_to_signed(mut alpha: f32) -> f32 {
    use core::f32::consts::PI;
    while alpha > PI {
        alpha -= 2.0 * PI;
    }
    while alpha < -PI {
        alpha += 2.0 * PI;
    }
    alpha
}

pub const NA_I16: i16 = i16::MAX;

impl AttitudeFrame {
    pub const PGN: u32 = 127257;
    const SCALE: f32 = 0.0001;

    pub fn encode(&self, data: &mut [u8]) {
        let yaw = self
            .yaw
            .map(|x| (wrap_rad_to_signed(x) / Self::SCALE) as i16)
            .unwrap_or(NA_I16);
        let pitch = self
            .pitch
            .map(|x| (wrap_rad_to_signed(x) / Self::SCALE) as i16)
            .unwrap_or(NA_I16);
        let roll = self
            .roll
            .map(|x| (wrap_rad_to_signed(x) / Self::SCALE) as i16)
            .unwrap_or(NA_I16);
        data[0] = self.sid;
        data[1..3].copy_from_slice(&yaw.to_le_bytes());
        data[3..5].copy_from_slice(&pitch.to_le_bytes());
        data[5..7].copy_from_slice(&roll.to_le_bytes());
    }
}

pub struct N2kId(u32);

impl N2kId {
    const MAX_PGN: u32 = 524287;

    pub const fn new(priority: u8, pgn: u32, source: u8) -> Self {
        assert!(priority <= 7, "Priority must be <= 7");
        assert!(pgn <= Self::MAX_PGN, "PGN must be <= MAX_PGN");

        Self(((priority as u32) << 26) | (pgn << 8) | source as u32)
    }

    pub fn from_can_id(can_id: CanId) -> Self {
        Self(can_id.raw())
    }

    pub fn from_raw_id(can_id: u32) -> Self {
        Self(can_id & 0x1FFFFFFF)
    }

    pub const fn as_can_id(&self) -> CanId {
        CanId::extended(self.0)
    }

    pub fn priority(&self) -> u8 {
        (self.0 >> 26) as u8
    }

    pub fn pgn(&self) -> u32 {
        (self.0 >> 8) & 0x3FFFF
    }

    pub fn source(&self) -> u8 {
        (self.0 & 0xff) as u8
    }
}
