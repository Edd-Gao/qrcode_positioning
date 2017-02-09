package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.droneState.DroneLanded;
import com.state_machine.core.droneState.DroneStateTracker;
import mavros_msgs.CommandTOLRequest;
import mavros_msgs.CommandTOLResponse;
import org.ros.exception.RemoteException;
import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import static java.lang.Math.abs;

/**
 * Created by firefly on 10/23/16.
 */
public class PX4LandAction extends Action{

    private DroneStateTracker stateTracker;
    private Duration timeOut;
    private ServiceClient<CommandTOLRequest, CommandTOLResponse> landService;


    public PX4LandAction(
            DroneStateTracker stateTracker,
            Duration timeOut,
            ServiceClient<CommandTOLRequest,CommandTOLResponse> landService
    ){
        this.stateTracker = stateTracker;
        this.timeOut = timeOut;
        this.landService = landService;
    }

    @Override
    public ActionStatus enterAction(Time time){
        double currentLongitude = stateTracker.getLongitude();
        double currentLattitude = stateTracker.getLattitude();

        status = ActionStatus.Inactive;

        if(stateTracker.getDroneLanded() == DroneLanded.OnGround){
            return ActionStatus.Success;
        }else if(stateTracker.getDroneLanded() == DroneLanded.InAir){
            if(!landService.isConnected())
                return ActionStatus.ConnectionFailure;

            CommandTOLRequest message = landService.newMessage();
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
            landService.call(message,listener);

            timeStamp = time;

            status = ActionStatus.Inactive;
            return ActionStatus.Success;
        }else{
            return ActionStatus.Failure;
        }
    }

    @Override
    public ActionStatus loopAction(Time time) {
        if(stateTracker.getDroneLanded() == DroneLanded.OnGround) {
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
        return "PX4LandAction";
    }
}
