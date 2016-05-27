package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.providers.FileProvider;
import com.state_machine.core.providers.RosPublisherProvider;
import mavros_msgs.CommandBoolRequest;
import mavros_msgs.CommandBoolResponse;
import mavros_msgs.OverrideRCIn;
import org.ros.exception.RemoteException;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;

import java.util.Properties;

public class ArmAction extends Action {

    private DroneStateTracker stateTracker;
    private ServiceClient<CommandBoolRequest, CommandBoolResponse> armingService;
    private FileProvider fileProvider;
    private RosPublisherProvider publisherProvider;
    private Properties config;
    private Publisher<OverrideRCIn> overrideRCInPublisher;

    public ArmAction(
            ServiceClient<CommandBoolRequest, CommandBoolResponse> armingService,
            DroneStateTracker stateTracker, FileProvider fileProvider,
            RosPublisherProvider publisherProvider
    ){
        super();
        this.stateTracker = stateTracker;
        this.armingService = armingService;
        this.fileProvider = fileProvider;
        this.publisherProvider = publisherProvider;
        this.config = fileProvider.getConfig();
        this.overrideRCInPublisher = publisherProvider.getOverrideRCInPubliser();
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
