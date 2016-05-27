package com.state_machine.core.states;

import com.state_machine.core.actions.Action;
import com.state_machine.core.droneState.DroneLanded;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.providers.ActionProvider;
import com.state_machine.core.providers.FlyToActionFactory;
import com.state_machine.core.actions.util.Waypoint;
import com.state_machine.core.states.util.ErrorType;
import com.state_machine.core.states.util.Failure;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.logging.Log;
import org.ros.exception.RemoteException;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import java.util.List;
import java.util.Queue;

public class WaypointState extends State {

    FlyToActionFactory factory;
    Queue<Waypoint> waypoints;
    Waypoint objective;
    ActionProvider actionProvider;

    public WaypointState(Queue<Waypoint> waypoints,
                         ActionProvider actionProvider,
                         ServiceClient<SetModeRequest, SetModeResponse> setModeService,
                         Log log) {
        super(actionProvider, setModeService, log);
        this.factory = actionProvider;
        this.waypoints = waypoints;
        this.actionProvider = actionProvider;
        prerequisites.add(actionProvider.getArmAction());
        prerequisites.add(actionProvider.getTakeoffAction());
    }

    protected void chooseNextAction(Time time){
            if (!waypoints.isEmpty()) {
                objective = waypoints.remove();
            }
            currentAction = factory.getFlyToAction(objective);
    }

    public boolean isIdling() { return waypoints.isEmpty(); }

    public boolean isSafeToExit(){
        return true;
    }

    @Override public void enterAction(){
        SetModeRequest request = setModeService.newMessage();
        request.setCustomMode("STABILIZE");
        setModeService.call(request, new ServiceResponseListener<SetModeResponse>() {
            @Override
            public void onSuccess(SetModeResponse setModeResponse) {

            }

            @Override
            public void onFailure(RemoteException e) {
                log.warn("Failed to set mode", e);
                lastFailure = new Failure(ErrorType.ConnectionFailure, currentTime);
            }
        });
        super.enterAction();
    }
}
