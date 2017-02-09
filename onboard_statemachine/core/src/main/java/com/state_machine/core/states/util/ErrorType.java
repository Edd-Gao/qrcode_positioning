package com.state_machine.core.states.util;

public enum ErrorType {
    NoError,
    BatteryLow,
    ConnectionFailure,
    ActionFailure,
    ControllerSignalLoss,
    MotorFailure,
    DangerousPosition
}
