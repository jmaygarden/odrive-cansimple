mod error;

pub use error::{ AxisError, EncoderError, MotorError };

pub type AxisId = u16;

#[repr(u32)]
#[derive(Debug, PartialEq, Eq, FromPrimitive)]
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
#[derive(Debug, FromPrimitive)]
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
}

#[repr(u32)]
#[derive(Debug, FromPrimitive)]
pub enum ControlMode {
    VoltageControl = 0,
    CurrentControl = 1,
    VelocityControl = 2,
    PositionControl = 3
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