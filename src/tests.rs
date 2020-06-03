use crate::Connection;

#[test]
fn test_nonexistant_device() {
    assert!(Connection::new("invalid").is_err());
}

#[cfg(feature = "vcan_tests")]
mod vcan_tests {

    #[test]
    fn test_vcan_open() {
        assert!(Connection::new("vcan0").is_ok());
    }

    #[test]
    fn test_vcan_loopback() {
        let frame = FrameBuilder::new(0, Command::OdriveHeartbeat)
            .arg0(AxisError::ERROR_NONE.bits())
            .arg1(AxisState::Idle as u32)
            .finalize()
            .unwrap();

        let s0 = Connection::new("vcan0").unwrap();
        let s1 = Connection::new("vcan0").unwrap();

        s0.write(&frame).unwrap();

        assert!(match s1.read().unwrap() {
            Signal::Heartbeat { id, error, state } => {
                id == 0 && error == AxisError::ERROR_NONE && state == AxisState::Idle
            }
            _ => false,
        });
    }
}
