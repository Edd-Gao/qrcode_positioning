package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.actions.util.Waypoint;
import com.state_machine.core.droneState.DroneLanded;
import com.state_machine.core.droneState.DroneStateTracker;
import mavros_msgs.CommandTOLRequest;
import mavros_msgs.CommandTOLResponse;
import org.ros.RosRun;
import org.ros.exception.RemoteException;
import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import static java.lang.Math.abs;

/**
 * Created by firefly on 10/21/16.
 */
public class PX4TakeoffAction extends Action{

    private DroneStateTracker stateTracker;
    private double target_heightm;
    private Duration timeOut;
    private ServiceClient<CommandTOLRequest, CommandTOLResponse> takeoffService;


    public PX4TakeoffAction(
            DroneStateTracker stateTracker,
            double target_heightm,
            Duration timeOut,
            ServiceClient<CommandTOLRequest,CommandTOLResponse> takeoffService
    ){
        this.stateTracker = stateTracker;
        this.target_heightm = target_heightm;
        this.timeOut = timeOut;
        this.takeoffService = takeoffService;
    }

    @Override
    public ActionStatus enterAction(Time time){
        double currentLongitude = stateTracker.getLongitude();
        double currentLattitude = stateTracker.getLattitude();
        double targetAltitude = stateTracker.getAltitude() + target_heightm;

        status = ActionStatus.Inactive;

        if(stateTracker.getDroneLanded() == DroneLanded.InAir){
            return ActionStatus.Success;
        }else if(stateTracker.getDroneLanded() == DroneLanded.OnGround){
            if(!takeoffService.isConnected())
                return ActionStatus.ConnectionFailure;

            CommandTOLRequest message = takeoffService.newMessage();
            message.setAltitude((float)(targetAltitude));
            message.setLatitude((float)(currentLattitude));
            message.setLongitude((float)(currentLongitude));
            ServiceResponseListener<CommandTOLResponse> listener = new ServiceResponseListener<CommandTOLResponse>() {
                @Override
                public void onSuccess(CommandTOLResponse commandTOLResponse) {
                    status = ActionStatus.Success;
                }

                @Override
                public void onFailure(RemoteException e) {
                    status = ActionStatus.Failure;
                }
            };
            takeoffService.call(message,listener);

            timeStamp = time;

            status = ActionStatus.Inactive;
            return ActionStatus.Success;
        }else{
            return ActionStatus.Failure;
        }
    }

    @Override
    public ActionStatus loopAction(Time time) {
        if(stateTracker.getDroneLanded() == DroneLanded.InAir
                && abs(stateTracker.getAltitude() - target_heightm) < 0.5){
            status = ActionStatus.Success;
            return status;
        }else if(time.subtract(timeStamp).compareTo(timeOut) >= 0){
            status = ActionStatus.Failure;
            return status;
        }else{
            status = ActionStatus.Waiting;
            return status;
        }
    }

    public String toString() {
        return "PX4TakeoffAction";
    }
}
