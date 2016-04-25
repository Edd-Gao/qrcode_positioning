package com.state_machine.core.states;

public enum ErrorType {
    NoError,
    BatteryLow,
    ConnectionFailure,
    ActionFailure,
    ControllerSignalLoss,
    MotorFailure,
    DangerousPosition
}
