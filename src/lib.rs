//! # ODrive CANSimple
//!
//! `odrive-cansimple` is a Rust client library for communicating with ODrive
//! motor controllers on CAN bus using the [CANSimple protocol](https://github.com/Wetmelon/ODrive/blob/feature/CAN/docs/can-protocol.md).
//!

#[macro_use]
extern crate bitflags;
extern crate byteorder;
extern crate num;
extern crate socketcan;
#[macro_use]
extern crate num_derive;

mod enumerations;
mod utils;

pub use crate::enumerations::{
    AxisError, AxisId, AxisState, Command, ControlMode, EncoderError, InputMode, MotorError,
};
use crate::utils::{decode_id, FrameBuilder};
use byteorder::{LittleEndian, ReadBytesExt};
use socketcan::{CANFrame, CANSocket, CANSocketOpenError};

#[cfg(test)]
mod tests;

/// Custom signal that may be received from an ODrive.
#[derive(Debug)]
pub struct CustomSignal {
    frame: CANFrame,
}

impl CustomSignal {
    pub fn id(&self) -> AxisId {
        ((self.frame.id() >> 5) & 0x3F) as AxisId
    }

    pub fn command(&self) -> Option<Command> {
        num::FromPrimitive::from_u32(self.frame.id() & 0x1F)
    }

    pub fn data(&self) -> &[u8] {
        return self.frame.data();
    }
}

/// Signals that may be received from an ODrive.
#[derive(Debug)]
pub enum Signal {
    EncoderCount {
        id: AxisId,
        shadow_count: i32,
        cpr_count: i32,
    },
    EncoderError {
        id: AxisId,
        error: EncoderError,
    },
    EncoderEstimates {
        id: AxisId,
        position: f32,
        velocity: f32,
    },
    Heartbeat {
        id: AxisId,
        error: AxisError,
        state: AxisState,
    },
    MotorCurrent {
        id: AxisId,
        setpoint: f32,
        measurement: f32,
    },
    MotorError {
        id: AxisId,
        error: MotorError,
    },
    VbusVoltage {
        id: AxisId,
        voltage: f32,
    },
    Unknown(CustomSignal),
}

/// Master CAN connection to one or more ODrive boards.
pub struct Connection {
    socket: CANSocket,
}

impl Connection {
    /// Returns an open CAN bus connection.
    ///
    /// Example:
    ///
    /// ```
    /// match odrive_cansimple::Connection::new("can0") {
    ///     Ok(conn) => (),
    ///     Err(error) => (),
    /// }
    /// ```
    pub fn new(ifname: &str) -> Result<Connection, CANSocketOpenError> {
        match CANSocket::open(ifname) {
            Ok(socket) => match socket.set_nonblocking(false) {
                Ok(()) => Ok(Connection { socket: socket }),
                Err(error) => Err(CANSocketOpenError::IOError(error)),
            },
            Err(error) => Err(error),
        }
    }

    /// Blocks until a a CAN frame is read from the bus.
    pub fn read(&self) -> std::io::Result<Signal> {
        match self.socket.read_frame() {
            Ok(frame) => {
                let (node, command) = decode_id(frame);
                let mut reader = std::io::Cursor::new(frame.data());

                match command {
                    Command::GetEncoderCount => {
                        let shadow_count = reader.read_i32::<LittleEndian>().unwrap();
                        let cpr_count = reader.read_i32::<LittleEndian>().unwrap();

                        Ok(Signal::EncoderCount {
                            id: node,
                            shadow_count: shadow_count,
                            cpr_count: cpr_count,
                        })
                    }

                    Command::GetEncoderError => {
                        let error = match EncoderError::from_bits(
                            reader.read_u32::<LittleEndian>().unwrap(),
                        ) {
                            Some(value) => value,
                            None => EncoderError::ERROR_NONE,
                        };

                        Ok(Signal::EncoderError {
                            id: node,
                            error: error,
                        })
                    }

                    Command::GetEncoderEstimates => {
                        let position = reader.read_f32::<LittleEndian>().unwrap();
                        let velocity = reader.read_f32::<LittleEndian>().unwrap();

                        Ok(Signal::EncoderEstimates {
                            id: node,
                            position: position,
                            velocity: velocity,
                        })
                    }

                    Command::OdriveHeartbeat => {
                        let error = match AxisError::from_bits(
                            reader.read_u32::<LittleEndian>().unwrap(),
                        ) {
                            Some(value) => value,
                            None => AxisError::ERROR_NONE,
                        };
                        let state = match num::FromPrimitive::from_u32(
                            reader.read_u32::<LittleEndian>().unwrap(),
                        ) {
                            Some(value) => value,
                            None => AxisState::Undefined,
                        };

                        Ok(Signal::Heartbeat {
                            id: node,
                            error: error,
                            state: state,
                        })
                    }

                    Command::GetIq => {
                        let setpoint = reader.read_f32::<LittleEndian>().unwrap();
                        let measurement = reader.read_f32::<LittleEndian>().unwrap();

                        Ok(Signal::MotorCurrent {
                            id: node,
                            setpoint: setpoint,
                            measurement: measurement,
                        })
                    }

                    Command::GetMotorError => {
                        let error =
                            match MotorError::from_bits(reader.read_u32::<LittleEndian>().unwrap())
                            {
                                Some(value) => value,
                                None => MotorError::ERROR_NONE,
                            };

                        Ok(Signal::MotorError {
                            id: node,
                            error: error,
                        })
                    }

                    Command::GetVbusVoltage => {
                        let voltage = reader.read_f32::<LittleEndian>().unwrap();

                        Ok(Signal::VbusVoltage {
                            id: node,
                            voltage: voltage,
                        })
                    }

                    _ => Ok(Signal::Unknown(CustomSignal { frame: frame })),
                }
            }
            Err(error) => Err(error),
        }
    }

    fn write(&self, frame: &CANFrame) -> std::io::Result<()> {
        self.socket.write_frame(&frame)
    }

    fn write_command(&self, id: AxisId, command: Command) -> std::io::Result<()> {
        let frame = FrameBuilder::new(id as u32, command).finalize().unwrap();

        self.write(&frame)
    }

    pub fn emergency_stop(&self, id: AxisId) -> std::io::Result<()> {
        self.write_command(id, Command::OdriveEstop)
    }

    pub fn reboot(&self, id: AxisId) -> std::io::Result<()> {
        self.write_command(id, Command::ResetOdrive)
    }

    pub fn clear_errors(&self, id: AxisId) -> std::io::Result<()> {
        self.write_command(id, Command::ClearErrors)
    }

    pub fn start_anticogging(&self, id: AxisId) -> std::io::Result<()> {
        self.write_command(id, Command::StartAnticogging)
    }

    pub fn set_axis_node_id(&self, id: AxisId, new_id: AxisId) -> std::io::Result<()> {
        let frame = FrameBuilder::new(id as u32, Command::SetAxisNodeId)
            .arg0(new_id as u32)
            .finalize()
            .unwrap();

        self.write(&frame)
    }

    pub fn set_axis_requested_state(&self, id: AxisId, state: AxisState) -> std::io::Result<()> {
        let frame = FrameBuilder::new(id as u32, Command::SetAxisRequestedState)
            .arg0(state as u32)
            .finalize()
            .unwrap();

        self.write(&frame)
    }

    /// Set the current setpoint to the given value in amps with 0.01 A granularity.
    pub fn set_input_current(&self, id: AxisId, value: f32) -> std::io::Result<()> {
        let frame = FrameBuilder::new(id as u32, Command::SetInputCurrent)
            .arg0((100. * value + 0.5) as i32 as u32)
            .finalize()
            .unwrap();

        self.write(&frame)
    }

    pub fn set_velocity_limit(&self, id: AxisId, value: f32) -> std::io::Result<()> {
        let frame = FrameBuilder::new(id as u32, Command::SetVelLimit)
            .arg0(u32::from_le_bytes(value.to_le_bytes()))
            .finalize()
            .unwrap();

        self.write(&frame)
    }

    pub fn set_controller_modes(
        &self,
        id: AxisId,
        control_mode: ControlMode,
    ) -> std::io::Result<()> {
        let frame = FrameBuilder::new(id as u32, Command::SetControllerModes)
            .arg0(control_mode as u32)
            .arg1(InputMode::PassThrough as u32)
            .finalize()
            .unwrap();

        self.write(&frame)
    }

    fn request(&self, id: AxisId, command: Command) -> std::io::Result<()> {
        let frame = FrameBuilder::new(id as u32, command)
            .rtr(true)
            .finalize()
            .unwrap();

        self.write(&frame)
    }

    pub fn get_motor_error(&self, id: AxisId) -> std::io::Result<()> {
        self.request(id, Command::GetMotorError)
    }

    pub fn get_encoder_error(&self, id: AxisId) -> std::io::Result<()> {
        self.request(id, Command::GetEncoderError)
    }

    pub fn get_sensorless_error(&self, id: AxisId) -> std::io::Result<()> {
        self.request(id, Command::GetSensorlessError)
    }

    pub fn get_encoder_estimates(&self, id: AxisId) -> std::io::Result<()> {
        self.request(id, Command::GetEncoderEstimates)
    }

    pub fn get_encoder_count(&self, id: AxisId) -> std::io::Result<()> {
        self.request(id, Command::GetEncoderCount)
    }

    pub fn get_iq(&self, id: AxisId) -> std::io::Result<()> {
        self.request(id, Command::GetIq)
    }

    pub fn get_sensorless_estimate(&self, id: AxisId) -> std::io::Result<()> {
        self.request(id, Command::GetSensorlessEstimates)
    }

    pub fn get_vbus_voltage(&self, id: AxisId) -> std::io::Result<()> {
        self.request(id, Command::GetVbusVoltage)
    }
}
