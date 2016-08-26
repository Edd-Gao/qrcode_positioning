package com.state_machine.core.providers;

import mavros_msgs.*;
import hover_controller_msgs.*;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;

public class RosServiceProvider {

    private ServiceClient<CommandBoolRequest, CommandBoolResponse> armingService;
    private ServiceClient<SetModeRequest, SetModeResponse> setFCUModeService;
    private ServiceClient<ParamPullRequest, ParamPullResponse> paramPullService;
    private ServiceClient<SwitchModeRequest,SwitchModeResponse> setHoverControllerModeService;
    private ServiceClient<WayPointRequest, WayPointResponse> setHoverControllerWayPointService;

    public RosServiceProvider(ConnectedNode node) throws ServiceNotFoundException {
        armingService = node.newServiceClient("mavros/cmd/arming", CommandBool._TYPE);
        setHoverControllerModeService = node.newServiceClient("hovercontroller/switchmode",SwitchMode._TYPE);
        setHoverControllerWayPointService = node.newServiceClient("hovercontroller/setnewwaypoint", hover_controller_msgs.WayPoint._TYPE );
        setFCUModeService = node.newServiceClient("mavros/set_mode", SetMode._TYPE);
        paramPullService = node.newServiceClient("mavros/param/pull",ParamPull._TYPE);
    }

    public ServiceClient<CommandBoolRequest, CommandBoolResponse> getArmingService() {
        return armingService;
    }

    public ServiceClient<SetModeRequest, SetModeResponse> getSetFCUModeService(){
        return setFCUModeService;
    }

    public ServiceClient<SwitchModeRequest, SwitchModeResponse> getSetHoverControllerModeService(){ return setHoverControllerModeService; };

    public ServiceClient<WayPointRequest, WayPointResponse> getSetHoverControllerWayPointService() { return setHoverControllerWayPointService;};

    public ServiceClient<ParamPullRequest, ParamPullResponse> getParamPullService() {return paramPullService;}

    public boolean isConnected(){
        return armingService.isConnected()
                && setHoverControllerModeService.isConnected()
                && setHoverControllerWayPointService.isConnected()
                && setFCUModeService.isConnected()
                && paramPullService.isConnected();
    }

}
