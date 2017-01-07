package com.state_machine.core.providers;

import mavros_msgs.ParamPullRequest;
import mavros_msgs.ParamPullResponse;
import org.apache.commons.logging.Log;
import org.ros.exception.RemoteException;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.*;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

/**
 * Created by parallels on 5/23/16.
 */
public class RosParamProvider {
    private ParameterTree params;
    private ServiceClient<ParamPullRequest, ParamPullResponse> paramPullService;
    private Log log;

    public RosParamProvider(ConnectedNode node,
                            RosServiceProvider rosServiceProvider,
                            Log log){
        this.paramPullService = rosServiceProvider.getParamPullService();
            ParamPullRequest request = paramPullService.newMessage();
            request.setForcePull(false);
            paramPullService.call(request, new ServiceResponseListener<ParamPullResponse>() {
                @Override
                public void onSuccess(ParamPullResponse paramPullResponse) {
                }

                @Override
                public void onFailure(RemoteException e) {
                }
            });
        params=node.getParameterTree();
    }

    public String getFlightScriptPath(){ return params.getString("state_machine/FlightScriptPath");}

    public String getPropertiesPath(){ return params.getString("state_machine/PropertiesPath");}

    public double getD0(){return params.getDouble("~/DecentralizedAction/D0",12.0);}

    public double getC(){return params.getDouble("~/DecentralizedAction/C", 0.2);}

    public double getLamda(){return params.getDouble("~/DecentralizedAction/Lamda", 1e-3);}

    public int getHp(){return params.getInteger("~/DecentralizedAction/Hp",3);}

    public int getHu(){return params.getInteger("~/DecentralizedAction/Hu", 2);}

    public double getTs(){return params.getDouble("~/DecentralizedAction/Ts", 0.10);}

    public double getVmax(){return params.getDouble("~/DecentralizedAction/Vmax", 3.5);}

}
