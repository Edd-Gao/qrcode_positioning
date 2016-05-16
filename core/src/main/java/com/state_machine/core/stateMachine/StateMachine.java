package com.state_machine.core.stateMachine;


import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.states.util.ErrorType;
import com.state_machine.core.states.State;
import com.state_machine.core.states.util.StateException;
import com.state_machine.core.states.util.StateHandle;
import org.apache.commons.logging.Log;
import org.ros.message.Time;

public class StateMachine {

    private State currentState;
    private State emergencyLanding;
    private State manualControl;
    private Log logger;
    private DroneStateTracker tracker;

    public StateMachine(State initialState, State emergencyLanding, State manualControl, Log log, DroneStateTracker tracker){
        this.emergencyLanding = emergencyLanding;
        this.manualControl = manualControl;
        this.logger = log;
        this.tracker = tracker;
        currentState = initialState;
        initialState.enterAction();
    }

    public void update(Time time){
        ErrorType error = failsafeCheck();
        switch (error) {
            case NoError:
                currentState.update(time);
                break;
            case ConnectionFailure:
                if (currentState != manualControl && currentState != emergencyLanding) {
                    logger.warn("Switching to manual control due to losing the wireless connection");
                    forceState(manualControl);
                }
                break;
            case DangerousPosition:
                if (currentState != manualControl && currentState != emergencyLanding) {
                    logger.warn("Switching to manual control due to perceived danger");
                    forceState(manualControl);
                }
                break;
            case BatteryLow:
                if (currentState != emergencyLanding) {
                    logger.warn("Starting emergency landing due to critical battery level");
                    forceState(emergencyLanding);
                }
                break;
            case ControllerSignalLoss:
                if (currentState != emergencyLanding) {
                    logger.warn("Starting emergency landing due to losing the signal of the manual controller");
                    forceState(emergencyLanding);
                }
                break;
            case MotorFailure:
                if (currentState != emergencyLanding) {
                    logger.warn("Starting emergency landing due to motor malfunction");
                    forceState(emergencyLanding);
                }
                break;
            case ActionFailure:
                if (currentState != manualControl && currentState != emergencyLanding) {
                    logger.warn("Switching to manual control due to " + currentState.getCurrentAction().toString() + " failing");
                    forceState(manualControl);
                }
                break;
        }
    }

    private ErrorType failsafeCheck(){
        if (currentState.getLastFailure() != null && currentState.getLastFailure().getError() != ErrorType.NoError){
            return currentState.getLastFailure().getError();
        }
        else if (tracker.getRemaining() < 0.05) return ErrorType.BatteryLow;
        else if (!currentState.isConnected()) return ErrorType.ConnectionFailure;

        return ErrorType.NoError;
    }

    public void setState(State newState) throws StateException {
        if(currentState.isSafeToExit()) {
            forceState(newState);
        }
        else {
            throw new StateException("The current state is unsafe to switch from!");
        }
    }

    public void forceState(State newState) {
        currentState.exitAction();
        currentState = newState;
        newState.enterAction();
    }

    public StateHandle getCurrentState(){
        return currentState;
    }
}
