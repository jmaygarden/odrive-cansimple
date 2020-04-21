bitflags! {
    #[derive(Default)]
    pub struct AxisError: u32 {
        const ERROR_NONE = 0x00;
        const ERROR_INVALID_STATE = 0x01;
        const ERROR_DC_BUS_UNDER_VOLTAGE = 0x02;
        const ERROR_DC_BUS_OVER_VOLTAGE = 0x04;
        const ERROR_CURRENT_MEASUREMENT_TIMEOUT = 0x08;
        const ERROR_BRAKE_RESISTOR_DISARMED = 0x10;
        const ERROR_MOTOR_DISARMED = 0x20;
        const ERROR_MOTOR_FAILED = 0x40;
        const ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x80;
        const ERROR_ENCODER_FAILED = 0x100;
        const ERROR_CONTROLLER_FAILED = 0x200;
        const ERROR_POS_CTRL_DURING_SENSORLESS = 0x400;
        const ERROR_WATCHDOG_TIMER_EXPIRED = 0x800;
        const ERROR_MIN_ENDSTOP_PRESSED = 0x1000;
        const ERROR_MAX_ENDSTOP_PRESSED = 0x2000;
        const ERROR_ESTOP_REQUESTED = 0x4000;
        const ERROR_DC_BUS_UNDER_CURRENT = 0x8000;
        const ERROR_DC_BUS_OVER_CURRENT = 0x10000;
        const ERROR_HOMING_WITHOUT_ENDSTOP = 0x20000;
    }
}

bitflags! {
    #[derive(Default)]
    pub struct EncoderError: u32 {
        const ERROR_NONE = 0x00;
        const ERROR_UNSTABLE_GAIN = 0x01;
        const ERROR_CPR_OUT_OF_RANGE = 0x02;
        const ERROR_NO_RESPONSE = 0x04;
        const ERROR_UNSUPPORTED_ENCODER_MODE = 0x08;
        const ERROR_ILLEGAL_HALL_STATE = 0x10;
        const ERROR_INDEX_NOT_FOUND_YET = 0x20;
    }
}

bitflags! {
    #[derive(Default)]
    pub struct MotorError: u32 {
        const ERROR_NONE = 0x0000;
        const ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x0001;
        const ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x0002;
        const ERROR_ADC_FAILED = 0x0004;
        const ERROR_DRV_FAULT = 0x0008;
        const ERROR_CONTROL_DEADLINE_MISSED = 0x0010;
        const ERROR_NOT_IMPLEMENTED_MOTOR_TYPE = 0x0020;
        const ERROR_BRAKE_CURRENT_OUT_OF_RANGE = 0x0040;
        const ERROR_MODULATION_MAGNITUDE = 0x0080;
        const ERROR_BRAKE_DEADTIME_VIOLATION = 0x0100;
        const ERROR_UNEXPECTED_TIMER_CALLBACK = 0x0200;
        const ERROR_CURRENT_SENSE_SATURATION = 0x0400;
        const ERROR_INVERTER_OVER_TEMP = 0x0800;
        const ERROR_CURRENT_UNSTABLE = 0x1000;
    }
}
