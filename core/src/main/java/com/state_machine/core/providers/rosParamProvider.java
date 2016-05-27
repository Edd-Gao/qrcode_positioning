package com.state_machine.core.providers;

import com.state_machine.core.states.util.ErrorType;
import com.state_machine.core.states.util.Failure;
import mavros_msgs.ParamPullRequest;
import mavros_msgs.ParamPullResponse;
import mavros_msgs.SetModeResponse;
import org.ros.exception.RemoteException;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.*;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

/**
 * Created by parallels on 5/23/16.
 */
public class rosParamProvider {
    private ParameterTree params;
    RosServiceProvider rosServiceProvider;
    private ServiceClient<ParamPullRequest, ParamPullResponse> paramPullService;

    public rosParamProvider(ConnectedNode node,
                            RosServiceProvider rosServiceProvider){
        this.rosServiceProvider = rosServiceProvider;
        this.paramPullService = rosServiceProvider.getParamPullService();
        try {
            ParamPullRequest request = paramPullService.newMessage();
            request.setForcePull(true);
            paramPullService.call(request, new ServiceResponseListener<ParamPullResponse>() {
                @Override
                public void onSuccess(ParamPullResponse paramPullResponse) {

                }

                @Override
                public void onFailure(RemoteException e) {
                    log.warn("Failed to set mode", e);
                    lastFailure = new Failure(ErrorType.ConnectionFailure, currentTime);
                }
            params = node.getParameterTree();

        }
    }

    public double getAC_A(){return params.getDouble("/");}

}
