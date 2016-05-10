package com.state_machine.core.states.util;

public interface StateHandle {

    boolean isSafeToExit();

    boolean isIdling();

    boolean isConnected();

    Failure getLastFailure();

    String toString();
}
