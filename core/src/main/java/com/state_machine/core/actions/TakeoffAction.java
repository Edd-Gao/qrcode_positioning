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

public class TakeoffAction extends Action {

    private DroneStateTracker stateTracker;
    private ServiceClient<SwitchModeRequest, SwitchModeResponse> hoverControllerSwitchModeService;
    private float target_heightcm;
    private Duration timeOut;
    private RosServerProvider rosServerProvider;



    public TakeoffAction(
            ServiceClient<SwitchModeRequest, SwitchModeResponse> hoverControllerSwitchModeService,
            DroneStateTracker stateTracker,
            float target_heightcm,
            RosServerProvider rosServerProvider, Duration timeOut){
        this.hoverControllerSwitchModeService = hoverControllerSwitchModeService;
        this.stateTracker = stateTracker;
        this.target_heightcm = target_heightcm;
        this.rosServerProvider = rosServerProvider;
        this.timeOut = timeOut;
    }

    @Override
    public ActionStatus enterAction() {
        status = ActionStatus.Inactive;

        if(stateTracker.getDroneLanded() == DroneLanded.InAir){
            return ActionStatus.Success;
        }
        else{
            if(!hoverControllerSwitchModeService.isConnected()) return ActionStatus.ConnectionFailure;

            SwitchModeRequest message = hoverControllerSwitchModeService.newMessage();
            message.setNewMode("TAKEOFF");
            message.setTargetHeight(target_heightcm);

            ServiceResponseListener<SwitchModeResponse> listener = new ServiceResponseListener<SwitchModeResponse>() {
                @Override
                public void onSuccess(SwitchModeResponse switchModeResponse) {
                    status = ActionStatus.Success;
                    timeStamp = timeProvider.getCurrentTime();
                }

                @Override
                public void onFailure(RemoteException e) { status = ActionStatus.Failure; }
            };
            hoverControllerSwitchModeService.call(message, listener);

            status = ActionStatus.Waiting;
            timeStamp = timeProvider.getCurrentTime();

            while(timeProvider.getCurrentTime().subtract(timeStamp).compareTo(enterTimeOut) <= 0 && status == ActionStatus.Waiting){}

            if (status == ActionStatus.Success){
                status = ActionStatus.Inactive;
                return ActionStatus.Success;
            }else{
                status = ActionStatus.Failure;
                return status;
            }
        }
    }

    public ActionStatus loopAction(Time time){
        if(stateTracker.getDroneLanded() == DroneLanded.InAir
                && rosServerProvider.getCurrentAction() == "TAKEOFF"
                && rosServerProvider.getActionFinished()){
            status = ActionStatus.Success;
            return status;
        }else if(timeProvider.getCurrentTime().subtract(timeStamp).compareTo(timeOut) >= 0){
            // if current time substracts timestamp is above timeout duration
            status = ActionStatus.Failure;
            return status;
        }else{
            status = ActionStatus.Waiting;
            return status;
        }

    }

    public String toString() {
        return "TakeoffAction";
    }
}
