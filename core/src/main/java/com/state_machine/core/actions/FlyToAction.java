package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.actions.util.Waypoint;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.providers.FileProvider;
import org.apache.commons.lang.NotImplementedException;
import org.ros.message.Time;

public class FlyToAction extends Action {

    private Waypoint objective;
    private DroneStateTracker stateTracker;
    private FileProvider fileProvider;

    public FlyToAction(Waypoint objective, DroneStateTracker stateTracker, FileProvider fileProvider){
        this.objective = objective;
        this.stateTracker = stateTracker;
        this.fileProvider = fileProvider;
    }

    public ActionStatus loopAction(Time time){
        throw new NotImplementedException();
    }
}
