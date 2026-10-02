pub const SENSOR_STREAM: &str = "sensor";
pub const FAULT_STREAM: &str = "fault";

pub fn derive_seed(master_seed: u64, subsystem: &str, robot_id: &str, source: &str) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64 ^ master_seed;
    for bytes in [subsystem.as_bytes(), robot_id.as_bytes(), source.as_bytes()] {
        for byte in bytes {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
        hash ^= 0xff;
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    splitmix64(hash)
}

pub fn sample_for_tick(stream_seed: u64, tick: u64, lane: u64) -> u64 {
    splitmix64(
        stream_seed
            .wrapping_add(tick.wrapping_mul(0x9e37_79b9_7f4a_7c15))
            .wrapping_add(lane.wrapping_mul(0xbf58_476d_1ce4_e5b9)),
    )
}

pub fn uniform_signed(value: u64) -> f64 {
    let unit = (value >> 11) as f64 * (1.0 / ((1_u64 << 53) as f64));
    unit * 2.0 - 1.0
}

fn splitmix64(mut value: u64) -> u64 {
    value = value.wrapping_add(0x9e37_79b9_7f4a_7c15);
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^ (value >> 31)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subsystem_streams_are_distinct_and_stable() {
        assert_ne!(
            derive_seed(7, SENSOR_STREAM, "r1", "default"),
            derive_seed(7, FAULT_STREAM, "r1", "default")
        );
        assert_eq!(
            derive_seed(7, SENSOR_STREAM, "r1", "default"),
            derive_seed(7, SENSOR_STREAM, "r1", "default")
        );
    }
}
