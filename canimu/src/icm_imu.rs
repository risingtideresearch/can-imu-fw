use core::{convert::Infallible, time::Duration};

use defmt::info;
use icm20948::{
    AccelDlpf, AccelFullScale, GyroDlpf, GyroFullScale, InterruptConfig, MagConfig, SpiInterface,
    sensors::{AccelConfig, GyroConfig},
};
use stm32_hal2::gpio::Pin;
use zencan_node::{common::sdo::AbortCode, object_dict::ObjectAccess as _};

use num_traits::float::Float;

use crate::{Delay, ICM_NOTIFY, notify_can_task, state, zencan};

fn gyro_rps_from_raw(gyro: (i16, i16, i16)) -> (f32, f32, f32) {
    const DEG2RAD: f32 = core::f32::consts::PI / 180.0;
    const GYRO_SCALE: f32 = 500.0 * DEG2RAD / 32768.0;
    (
        gyro.0 as f32 * GYRO_SCALE,
        gyro.1 as f32 * GYRO_SCALE,
        gyro.2 as f32 * GYRO_SCALE,
    )
}

pub const G_MPSS: f32 = 9.80665;

fn accel_mpss_from_raw(accel: (i16, i16, i16)) -> (f32, f32, f32) {
    const ACCEL_SCALE: f32 = 4.0 * G_MPSS / 32768.0; // m/s/s per LSB
    (
        accel.0 as f32 * ACCEL_SCALE,
        accel.1 as f32 * ACCEL_SCALE,
        accel.2 as f32 * ACCEL_SCALE,
    )
}

fn accel_g_from_raw(accel: (i16, i16, i16)) -> (f32, f32, f32) {
    const ACCEL_SCALE: f32 = 4.0 / 32768.0; // g per LSB
    (
        accel.0 as f32 * ACCEL_SCALE,
        accel.1 as f32 * ACCEL_SCALE,
        accel.2 as f32 * ACCEL_SCALE,
    )
}

fn offset_gyros(mut gyro: (i16, i16, i16)) -> (i16, i16, i16) {
    let gyro_x_offset = zencan::OBJECT2010.get(0).unwrap_or(0);
    let gyro_y_offset = zencan::OBJECT2010.get(1).unwrap_or(0);
    let gyro_z_offset = zencan::OBJECT2010.get(2).unwrap_or(0);
    gyro.0 = gyro.0.saturating_sub(gyro_x_offset);
    gyro.1 = gyro.1.saturating_sub(gyro_y_offset);
    gyro.2 = gyro.2.saturating_sub(gyro_z_offset);
    gyro
}

fn offset_accels(accel: (i16, i16, i16)) -> (i16, i16, i16) {
    let x_off = zencan::OBJECT2020.get(0).unwrap_or(0);
    let y_off = zencan::OBJECT2020.get(1).unwrap_or(0);
    let z_off = zencan::OBJECT2020.get(2).unwrap_or(0);
    (
        accel.0.saturating_sub(x_off),
        accel.1.saturating_sub(y_off),
        accel.2.saturating_sub(z_off),
    )
}

/// Rotate sensor readings into the desired coordinate frame.
fn rotate_sensors(raw: (i16, i16, i16)) -> (i16, i16, i16) {
    (-raw.1, -raw.0, -raw.2)
}

#[inline]
/// Function to store IMU data to object dict
///
/// Mainly motivated by the belief that it will create less panic sites and therefore smaller code
/// although this is an untested theory
fn store_icm20948_values(
    gyro: (i16, i16, i16),
    accel: (i16, i16, i16),
    mag: (i16, i16, i16),
) -> Result<(), AbortCode> {
    const MAG_SCALE: f32 = 1.0 / 0.015; // LSB per uT

    let gyro_rps = gyro_rps_from_raw(gyro);
    let accel_g = accel_g_from_raw(accel);

    zencan::OBJECT2000.set(0, gyro.0)?;
    zencan::OBJECT2000.set(1, gyro.1)?;
    zencan::OBJECT2000.set(2, gyro.2)?;
    zencan::OBJECT2001.set(0, gyro_rps.0)?;
    zencan::OBJECT2001.set(1, gyro_rps.1)?;
    zencan::OBJECT2001.set(2, gyro_rps.2)?;

    zencan::OBJECT2002.set(0, accel.0)?;
    zencan::OBJECT2002.set(1, accel.1)?;
    zencan::OBJECT2002.set(2, accel.2)?;
    zencan::OBJECT2003.set(0, accel_g.0)?;
    zencan::OBJECT2003.set(1, accel_g.1)?;
    zencan::OBJECT2003.set(2, accel_g.2)?;

    zencan::OBJECT2004.set(0, mag.0)?;
    zencan::OBJECT2004.set(1, mag.1)?;
    zencan::OBJECT2004.set(2, mag.2)?;
    zencan::OBJECT2005.set(0, mag.0 as f32 * MAG_SCALE)?;
    zencan::OBJECT2005.set(1, mag.1 as f32 * MAG_SCALE)?;
    zencan::OBJECT2005.set(2, mag.2 as f32 * MAG_SCALE)?;

    for i in 0..3 {
        zencan::OBJECT2000.set_event_flag(i)?;
        zencan::OBJECT2001.set_event_flag(i)?;
        zencan::OBJECT2002.set_event_flag(i)?;
        zencan::OBJECT2003.set_event_flag(i)?;
        zencan::OBJECT2004.set_event_flag(i)?;
        zencan::OBJECT2005.set_event_flag(i)?;
    }

    // Notify process task it needs to run
    notify_can_task();
    Ok(())
}

/// A task for reading data from the IMU
pub async fn imu_task<S: embedded_hal::spi::SpiBus>(spi: S, cs: Pin) -> Infallible {
    /// The number of samples to collect during gyro calibration
    const GYRO_CAL_SAMPLES: usize = 200;
    const OUTPUT_PERIOD: f32 = 0.1;
    const FSDIV: u16 = 11;
    let device = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let mut imu = icm20948::Icm20948Driver::new(SpiInterface::new(device)).unwrap();

    let mut cal_sample_counter: Option<usize> = None;
    let mut gyro_accum = [0i32, 0, 0];
    let sample_period = FSDIV as f32 / 1125.0;

    let mut filter = state::IcmFilter::new(sample_period, OUTPUT_PERIOD);

    let accel_config = AccelConfig {
        full_scale: AccelFullScale::G4,
        dlpf: AccelDlpf::Hz12,
        dlpf_enable: true,
        sample_rate_div: FSDIV - 1,
    };
    let gyro_config = GyroConfig {
        full_scale: GyroFullScale::Dps500,
        dlpf: GyroDlpf::Hz12,
        dlpf_enable: true,
        sample_rate_div: FSDIV as u8 - 1,
    };
    let mag_config = MagConfig {
        mode: icm20948::MagMode::Continuous10Hz,
    };

    let irq_config = InterruptConfig {
        raw_data_ready: true,
        ..Default::default()
    };

    if let Err(_) = imu.init(&mut Delay {}) {
        defmt::error!("Error initializing ICM");
    }
    if let Err(_) = imu.init_magnetometer(mag_config, &mut Delay {}) {
        defmt::error!("Error from init_magnetometer");
    }
    if let Err(_) = imu.configure_accelerometer(accel_config) {
        defmt::error!("Error configuring accel");
    }
    if let Err(_) = imu.configure_gyroscope(gyro_config) {
        defmt::error!("Error configuring gyro");
    }
    if let Err(_) = imu.configure_interrupts(&irq_config) {
        defmt::error!("Error configuring interrupts");
    }

    loop {
        // Wait for a notification
        lilos::time::with_timeout(Duration::from_millis(100), ICM_NOTIFY.until_next()).await;
        // Check status flag, so see if new data is actually available
        let flag = imu
            .device_mut()
            .int_status_1()
            .read()
            .unwrap()
            .raw_data_0_rdy_int();
        if !flag {
            info!("NO FLAG");
            continue;
        }

        let accel = rotate_sensors(imu.read_accelerometer_raw().unwrap_or((
            i16::MAX,
            i16::MAX,
            i16::MAX,
        )));
        let gyro = rotate_sensors(imu.read_gyroscope_raw().unwrap_or((
            i16::MAX,
            i16::MAX,
            i16::MAX,
        )));
        let mag = imu
            .read_magnetometer_raw()
            .unwrap_or((i16::MAX, i16::MAX, i16::MAX));

        // Check if calibration is in progress or requested
        let calibration_control = zencan::OBJECT2011.get_value();

        if calibration_control & (1 << 0) != 0 {
            cal_sample_counter = Some(0);
            gyro_accum = [0, 0, 0];
            // Set calibration in progress bit
            zencan::OBJECT2011.set_value(1 << 1);
        }

        if cal_sample_counter.is_some() {
            // In progress cal
            gyro_accum[0] += gyro.0 as i32;
            gyro_accum[1] += gyro.1 as i32;
            gyro_accum[2] += gyro.2 as i32;
            *cal_sample_counter.as_mut().unwrap() += 1;

            if cal_sample_counter.unwrap() == GYRO_CAL_SAMPLES {
                for i in 0..3 {
                    zencan::OBJECT2010
                        .set(i, (gyro_accum[i] / GYRO_CAL_SAMPLES as i32) as i16)
                        .unwrap();
                }
                cal_sample_counter = None;
                zencan::OBJECT2011.set_value(0);
            }
        }

        let gyro = offset_gyros(gyro);
        let accel = offset_accels(accel);
        store_icm20948_values(gyro, accel, mag).unwrap();

        // Do state estimate update
        let gyro_rps = gyro_rps_from_raw(gyro);
        let accel_mpss = accel_mpss_from_raw(accel);

        if let Some(state::FilterSample { pitch, roll, heave }) =
            filter.push_sample(accel_mpss, gyro_rps)
        {
            // Set float tilt
            zencan::OBJECT2111.set_pitch(pitch);
            zencan::OBJECT2111.set_roll(roll);
            // Set integer tilt
            zencan::OBJECT2110.set_pitch((pitch * 1e4).round() as i16);
            zencan::OBJECT2110.set_roll((roll * 1e4).round() as i16);
            // Set float heave
            zencan::OBJECT2101.set_value(heave);
            // Set integer heave
            zencan::OBJECT2100.set_value((heave * 100.0).round() as i16);

            zencan::OBJECT2110.set_event_flag(1).ok();
            zencan::OBJECT2110.set_event_flag(2).ok();
            zencan::OBJECT2111.set_event_flag(1).ok();
            zencan::OBJECT2111.set_event_flag(2).ok();
            zencan::OBJECT2100.set_event_flag(0).ok();
            zencan::OBJECT2101.set_event_flag(0).ok();
            notify_can_task();
        }
    }
}
