package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.actions.util.Waypoint;
import com.state_machine.core.droneState.DroneStateTracker;
import org.apache.commons.lang.NotImplementedException;
import org.ros.message.Time;

public class FlyToAction extends Action {

    private Waypoint objective;
    private DroneStateTracker stateTracker;

    public FlyToAction(Waypoint objective, DroneStateTracker stateTracker){
        this.objective = objective;
        this.stateTracker = stateTracker;
    }

    public ActionStatus loopAction(Time time){
        throw new NotImplementedException();
    }
}
