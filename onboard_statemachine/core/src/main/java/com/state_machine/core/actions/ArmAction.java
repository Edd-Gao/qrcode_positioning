package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.droneState.DroneStateTracker;
import mavros_msgs.CommandBoolRequest;
import mavros_msgs.CommandBoolResponse;
import org.ros.exception.RemoteException;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.apache.commons.logging.Log;

public class ArmAction extends Action {

    private DroneStateTracker stateTracker;
    private ServiceClient<CommandBoolRequest, CommandBoolResponse> armingService;
    private Log logger;

    public ArmAction(
            Log logger,
            ServiceClient<CommandBoolRequest, CommandBoolResponse> armingService,
            DroneStateTracker stateTracker
    ){
        super();
        this.logger = logger;
        this.stateTracker = stateTracker;
        this.armingService = armingService;
    }

    public ActionStatus loopAction(Time time){
        if (stateTracker.getArmed()) {
            return ActionStatus.Success;
        }
        else if (status == ActionStatus.Inactive) {
                if (!armingService.isConnected()) return ActionStatus.ConnectionFailure;

                CommandBoolRequest message = armingService.newMessage();
                message.setValue(true);
                ServiceResponseListener<CommandBoolResponse> listener = new ServiceResponseListener<CommandBoolResponse>() {
                    @Override
                    public void onSuccess(CommandBoolResponse commandBoolResponse) {
                        status = ActionStatus.Success;
                    }

                    @Override
                    public void onFailure(RemoteException e) {
                        status = ActionStatus.Failure;
                    }
                };
                armingService.call(message, listener);

                stateTracker.setLocalOrigin(stateTracker.getVisionPosition());
                //logger.warn("set local origin at "
                //        + stateTracker.getLocalOrigin()[0] + ","
                //        + stateTracker.getLocalOrigin()[1] + ","
                //       + stateTracker.getLocalOrigin()[2] + ".");

                status = ActionStatus.Waiting;
                return ActionStatus.Waiting;
        }
        else {
            return status;
        }
    }

    public String toString() {
        return "ArmAction";
    }
}
