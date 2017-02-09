package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.actions.util.Waypoint;
import com.state_machine.core.droneState.DroneLanded;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.providers.FileProvider;
import com.state_machine.core.providers.RosServerProvider;
import hover_controller_msgs.SwitchModeRequest;
import hover_controller_msgs.SwitchModeResponse;
import hover_controller_msgs.WayPointRequest;
import hover_controller_msgs.WayPointResponse;
import org.ros.exception.RemoteException;
import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class FlyToAction extends Action {

    private Waypoint objective;
    private DroneStateTracker stateTracker;
    private FileProvider fileProvider;
    private RosServerProvider rosServerProvider;
    private ServiceClient<SwitchModeRequest, SwitchModeResponse> hoverControllerSwitchModeService;
    private ServiceClient<WayPointRequest, WayPointResponse> hoverControllerWayPointService;
    private Duration timeOut;

    public FlyToAction(Waypoint objective, DroneStateTracker stateTracker, FileProvider fileProvider, RosServerProvider rosServerProvider, ServiceClient<SwitchModeRequest, SwitchModeResponse> hoverControllerSwitchModeService, ServiceClient<WayPointRequest, WayPointResponse> hoverControllerWayPointService, Duration timeOut){
        this.objective = objective;
        this.stateTracker = stateTracker;
        this.fileProvider = fileProvider;
        this.rosServerProvider = rosServerProvider;
        this.hoverControllerSwitchModeService = hoverControllerSwitchModeService;
        this.hoverControllerWayPointService = hoverControllerWayPointService;
        this.timeOut = timeOut;
    }

    public ActionStatus enterAction(Time time) {

        status = ActionStatus.Inactive;

        //if the drone is on ground, it can not enter Way Point mode
        if(stateTracker.getDroneLanded() == DroneLanded.OnGround){
            status = ActionStatus.Failure;
            return status;

        //else if the drone is in air, it enters way point mode
        }else if(stateTracker.getDroneLanded() == DroneLanded.InAir) {

            //if the drone is not in way point mode ,make it into way point mode
            if (!rosServerProvider.getCurrentAction().equals("WP")) {
                if (!hoverControllerSwitchModeService.isConnected()) return ActionStatus.ConnectionFailure;

                SwitchModeRequest message = hoverControllerSwitchModeService.newMessage();
                message.setNewMode("WP");
                message.setTargetHeight(0);

                ServiceResponseListener<SwitchModeResponse> listener = new ServiceResponseListener<SwitchModeResponse>() {
                    @Override
                    public void onSuccess(SwitchModeResponse switchModeResponse) {
                        status = ActionStatus.Success;
                    }

                    @Override
                    public void onFailure(RemoteException e) {
                        status = ActionStatus.Failure;
                    }
                };
                hoverControllerSwitchModeService.call(message, listener);

                //while (timeProvider.getCurrentTime().subtract(timeStamp).compareTo(enterTimeOut) <= 0 && status == ActionStatus.Waiting) {}
            }

            if(!hoverControllerWayPointService.isConnected()) return ActionStatus.ConnectionFailure;

            WayPointRequest wayPointRequest = hoverControllerWayPointService.newMessage();
            wayPointRequest.setX(objective.getX());
            wayPointRequest.setY(objective.getY());
            wayPointRequest.setZ(objective.getZ());

            ServiceResponseListener<WayPointResponse> listener = new ServiceResponseListener<WayPointResponse>() {
                @Override
                public void onSuccess(WayPointResponse wayPointResponse) {

                    status = ActionStatus.Success;
                }

                @Override
                public void onFailure(RemoteException e) {
                    status = ActionStatus.Failure;
                }
            };
            hoverControllerWayPointService.call(wayPointRequest, listener);

            timeStamp = time;

            //while (timeProvider.getCurrentTime().subtract(timeStamp).compareTo(enterTimeOut) <= 0 && status == ActionStatus.Waiting) {
            //}

                status = ActionStatus.Inactive;
                return ActionStatus.Success;

        }else{
            status = ActionStatus.Failure;
            return status;
        }
    }

    public ActionStatus loopAction(Time time){
        if(stateTracker.getDroneLanded() == DroneLanded.InAir
                && rosServerProvider.getCurrentAction().equals("WP")
                && rosServerProvider.getActionFinished()){
            status = ActionStatus.Success;
            //reset action status
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

    public String toString(){
        return "FlyToAction";
    }
}
