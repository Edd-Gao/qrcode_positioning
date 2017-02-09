package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.droneState.DroneLanded;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.providers.RosServerProvider;
import hover_controller_msgs.SwitchModeRequest;
import hover_controller_msgs.SwitchModeResponse;
import org.ros.exception.RemoteException;
import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class LandingAction extends Action {

    private DroneStateTracker stateTracker;
    private ServiceClient<SwitchModeRequest, SwitchModeResponse> hoverControllerSwitchModeService;
    private Duration timeOut;
    private RosServerProvider rosServerProvider;

    public LandingAction(
            ServiceClient<SwitchModeRequest, SwitchModeResponse> hoverControllerSwitchModeService,
            DroneStateTracker stateTracker,
            RosServerProvider rosServerProvider, Duration timeOut){
        this.hoverControllerSwitchModeService = hoverControllerSwitchModeService;
        this.stateTracker = stateTracker;
        this.rosServerProvider = rosServerProvider;
        this.timeOut = timeOut;
    }

    @Override
    public ActionStatus enterAction(Time time){
        status = ActionStatus.Inactive;

        if(stateTracker.getDroneLanded() == DroneLanded.OnGround){
            return ActionStatus.Success;
        }
        else{
            if(!hoverControllerSwitchModeService.isConnected()) return ActionStatus.ConnectionFailure;

            SwitchModeRequest message = hoverControllerSwitchModeService.newMessage();
            message.setNewMode("LAND");
            message.setTargetHeight(0);

            ServiceResponseListener<SwitchModeResponse> listener = new ServiceResponseListener<SwitchModeResponse>() {
                @Override
                public void onSuccess(SwitchModeResponse switchModeResponse) {
                    status = ActionStatus.Success;
                }

                @Override
                public void onFailure(RemoteException e) { status = ActionStatus.Failure; }
            };
            hoverControllerSwitchModeService.call(message, listener);

            timeStamp = time;

            //while(timeProvider.getCurrentTime().subtract(timeStamp).compareTo(enterTimeOut) <= 0 && status == ActionStatus.Waiting){}

            status = ActionStatus.Inactive;
            return ActionStatus.Success;

        }
    }

    public ActionStatus loopAction(Time time){
        if(stateTracker.getDroneLanded() == DroneLanded.OnGround
                && rosServerProvider.getCurrentAction().equals("LAND")
                && rosServerProvider.getActionFinished()){
            status = ActionStatus.Success;
            rosServerProvider.resetActionStatus();
            return status;
        }else if(time.subtract(timeStamp).compareTo(timeOut) >= 0){
            // if current time substracts timestamp is above timeout duration
            status = ActionStatus.Failure;
            return status;
        }else{
            status = ActionStatus.Waiting;
            return status;
        }

    }

    public String toString() {
        return "LandingAction";
    }
}
