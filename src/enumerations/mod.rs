mod error;

pub use error::{AxisError, EncoderError, MotorError};

pub type AxisId = u16;

#[repr(u32)]
#[derive(Debug, PartialEq, Eq, FromPrimitive, Copy, Clone)]
pub enum AxisState {
    Undefined,
    Idle,
    StartupSequence,
    FullCalibrationSequence,
    MotorCalibration,
    SensorlessControl,
    EncoderIndexSearch,
    EncoderOffsetCalibration,
    ClosedLoopControl,
    LockinSpin,
    EncoderDirFind,
    Homing,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone)]
pub enum Command {
    Undefined,
    OdriveHeartbeat,
    OdriveEstop,
    GetMotorError,
    GetEncoderError,
    GetSensorlessError,
    SetAxisNodeId,
    SetAxisRequestedState,
    SetAxisStartupConfig,
    GetEncoderEstimates,
    GetEncoderCount,
    SetControllerModes,
    SetInputPos,
    SetInputVel,
    SetInputCurrent,
    SetVelLimit,
    StartAnticogging,
    SetTrajVelLimit,
    SetTrajAccelLimits,
    SetTrajAPerCss,
    GetIq,
    GetSensorlessEstimates,
    ResetOdrive,
    GetVbusVoltage,
    ClearErrors,
    Custom(u32),
}

impl From<u32> for Command {
    fn from(n: u32) -> Command {
        match n {
            0 => Command::Undefined,
            1 => Command::OdriveHeartbeat,
            2 => Command::OdriveEstop,
            3 => Command::GetMotorError,
            4 => Command::GetEncoderError,
            5 => Command::GetSensorlessError,
            6 => Command::SetAxisNodeId,
            7 => Command::SetAxisRequestedState,
            8 => Command::SetAxisStartupConfig,
            9 => Command::GetEncoderEstimates,
            10 => Command::GetEncoderCount,
            11 => Command::SetControllerModes,
            12 => Command::SetInputPos,
            13 => Command::SetInputVel,
            14 => Command::SetInputCurrent,
            15 => Command::SetVelLimit,
            16 => Command::StartAnticogging,
            17 => Command::SetTrajVelLimit,
            18 => Command::SetTrajAccelLimits,
            19 => Command::SetTrajAPerCss,
            20 => Command::GetIq,
            21 => Command::GetSensorlessEstimates,
            22 => Command::ResetOdrive,
            23 => Command::GetVbusVoltage,
            24 => Command::ClearErrors,
            _ => Command::Custom(n),
        }
    }
}

impl From<Command> for u32 {
    fn from(command: Command) -> u32 {
        match command {
            Command::Undefined => 0,
            Command::OdriveHeartbeat => 1,
            Command::OdriveEstop => 2,
            Command::GetMotorError => 3,
            Command::GetEncoderError => 4,
            Command::GetSensorlessError => 5,
            Command::SetAxisNodeId => 6,
            Command::SetAxisRequestedState => 7,
            Command::SetAxisStartupConfig => 8,
            Command::GetEncoderEstimates => 9,
            Command::GetEncoderCount => 10,
            Command::SetControllerModes => 11,
            Command::SetInputPos => 12,
            Command::SetInputVel => 13,
            Command::SetInputCurrent => 14,
            Command::SetVelLimit => 15,
            Command::StartAnticogging => 16,
            Command::SetTrajVelLimit => 17,
            Command::SetTrajAccelLimits => 18,
            Command::SetTrajAPerCss => 19,
            Command::GetIq => 20,
            Command::GetSensorlessEstimates => 21,
            Command::ResetOdrive => 22,
            Command::GetVbusVoltage => 23,
            Command::ClearErrors => 24,
            Command::Custom(n) => n,
        }
    }
}

#[repr(u32)]
#[derive(Debug, FromPrimitive)]
pub enum ControlMode {
    VoltageControl = 0,
    CurrentControl = 1,
    VelocityControl = 2,
    PositionControl = 3,
}

#[repr(u32)]
#[derive(Debug, FromPrimitive)]
pub enum InputMode {
    Inactive,
    PassThrough,
    VelocityRamp,
    PositionFilter,
    MixChannels,
    TrapezoidalTrajectory,
    CurrentRamp,
    Mirror,
}
