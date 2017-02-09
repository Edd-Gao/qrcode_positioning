package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.ros.exception.RemoteException;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

/**
 * Created by parallels on 8/20/16.
 */
public class SetFCUModeAction extends Action{

    private ServiceClient<SetModeRequest, SetModeResponse> setModeService;
    private String newMode;
    private final byte baseMode = 0;

    public SetFCUModeAction(ServiceClient<SetModeRequest, SetModeResponse> setModeService,
                            String newMode){
        this.setModeService = setModeService;
        this.newMode = newMode;
    }


    @Override
    public ActionStatus loopAction(Time time) {
        if(status == ActionStatus.Inactive) {
            if(!setModeService.isConnected()) return ActionStatus.ConnectionFailure;

            SetModeRequest request = setModeService.newMessage();
            request.setBaseMode(baseMode);
            request.setCustomMode(newMode);

            ServiceResponseListener<SetModeResponse> listener = new ServiceResponseListener<SetModeResponse>() {
                @Override
                public void onSuccess(SetModeResponse setModeResponse) {
                    status = ActionStatus.Success;
                }

                @Override
                public void onFailure(RemoteException e) {
                    status = ActionStatus.Failure;
                }
            };
            setModeService.call(request, listener);

            status = ActionStatus.Waiting;
            return status;
        }else{
            return status;
        }
    }

    public String toString(){
        return "SetFCUModeAction";
    }
}
