use core::convert::Infallible;

use microstrain_inertial::{api::commands::imu_3dm::DescriptorRate, api::data::filter};
use num_quaternion::Q32;
use zencan_node::object_dict::ObjectAccess;

use crate::{enable_uart_tx_irq, notify_can_task, state::Cm7Filter, zencan};

use num_traits::float::Float;

pub async fn cm7_task() -> Infallible {
    let cm7 = &crate::CM7_IMU;

    /// The base rate of the IMU is 1kHz. This specifies the decimation factor for IMU output.
    const IMU_DECIMATION: u16 = 20;
    /// Computed IMU output sample period (sec)
    const SAMPLE_PERIOD: f32 = 1e-3 * IMU_DECIMATION as f32;
    /// Compute an output decimation to achieve 10Hz
    const OUTPUT_DECIMATION: usize = 100 / IMU_DECIMATION as usize;

    defmt::info!("Initializing CM7 IMU");
    // Configure the IMU with the messages we want to receive
    let f = cm7.set_message_format(
        filter::FILTER_DESCRIPTOR_SET,
        &[
            DescriptorRate {
                descriptor: filter::AttitudeQuaternion::DESCRIPTOR,
                decimation: IMU_DECIMATION,
            },
            DescriptorRate {
                descriptor: filter::LinearAccel::DESCRIPTOR,
                decimation: IMU_DECIMATION,
            },
        ],
    );
    // Kiiiiiind of hacky here. Get the future, which queues data bytes for transmit, then enable
    // the TX irq, which sends them, allowing the future to complete.
    // the CM7 driver really needs a callback...
    enable_uart_tx_irq();
    f.await.expect("Failed to configure CM7 message format");

    defmt::info!("Completed CM7 init");

    let mut heave_filter = Cm7Filter::new(SAMPLE_PERIOD);
    let mut sample_counter = 0;
    loop {
        let data_packet = match cm7.read_raw_data().await {
            Ok(p) => p,
            Err(e) => {
                defmt::error!("Error reading data packet: {:?}", e);
                continue;
            }
        };

        if data_packet.descriptor_set() != filter::FILTER_DESCRIPTOR_SET {
            continue;
        }

        let filter_packet = filter::FilterPacket::new(data_packet.payload());
        let mut quat = None;
        let mut accel = None;
        for field in filter_packet.fields() {
            match field {
                Ok(filter::FilterField::AttitudeQuaternion(att)) => quat = Some(att.q),
                Ok(filter::FilterField::LinearAccel(a)) => accel = Some(a.accel_m_s2),
                Err(e) => {
                    defmt::error!("Error parsing filter packet: {:?}", e);
                    continue;
                }
                _ => (),
            }
        }

        if quat.is_none() || accel.is_none() {
            // The IMU should ALWAYS send both fields in each filter frame, but just in case...
            defmt::error!("Did not get both attitude and linear accel fields");
            continue;
        }
        let accel = accel.unwrap();
        // Convert to tuple
        let accel = (accel.x, accel.y, accel.z);

        let quat = quat.unwrap();
        let quat = Q32::new(quat.w, quat.x, quat.y, quat.z)
            .normalize()
            .unwrap_or_default();
        let heave = heave_filter.push_sample(quat, accel);

        sample_counter += 1;
        if sample_counter == OUTPUT_DECIMATION {
            sample_counter = 0;
            let eulers = quat.to_euler_angles();
            defmt::info!("Accel: {} {} {}", accel.0, accel.1, accel.2);
            defmt::info!("CM7: {} {} {}", eulers.roll, eulers.pitch, heave);
            store_cm7_values(eulers.roll, eulers.pitch, heave);
        }
    }
}

fn store_cm7_values(roll: f32, pitch: f32, heave: f32) {
    let heave_fixed = (heave * 100.0).round() as i16;
    let roll_fixed = (roll * 1e4).round() as i16;
    let pitch_fixed = (pitch * 1e4).round() as i16;
    zencan::OBJECT2180.set_value(heave_fixed);
    zencan::OBJECT2181.set_value(heave);
    zencan::OBJECT2190.set_pitch(pitch_fixed);
    zencan::OBJECT2190.set_roll(roll_fixed);
    zencan::OBJECT2191.set_pitch(pitch);
    zencan::OBJECT2191.set_roll(roll);

    zencan::OBJECT2180.set_event_flag(0).ok();
    zencan::OBJECT2181.set_event_flag(0).ok();
    zencan::OBJECT2190.set_event_flag(1).ok();
    zencan::OBJECT2190.set_event_flag(2).ok();
    zencan::OBJECT2191.set_event_flag(1).ok();
    zencan::OBJECT2191.set_event_flag(2).ok();
    notify_can_task();
}
