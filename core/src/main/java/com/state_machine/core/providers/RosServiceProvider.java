package com.state_machine.core.providers;

import mavros_msgs.*;
import hover_controller_msgs.*;
import org.apache.commons.logging.Log;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;

public class RosServiceProvider {


    private ServiceClient<CommandBoolRequest, CommandBoolResponse> armingService;
    private ServiceClient<CommandTOLRequest, CommandTOLResponse> takeoffService;
    private ServiceClient<CommandTOLRequest, CommandTOLResponse> landService;
    private ServiceClient<SetModeRequest, SetModeResponse> setFCUModeService;
    private ServiceClient<ParamPullRequest, ParamPullResponse> paramPullService;
    private ServiceClient<SwitchModeRequest,SwitchModeResponse> setHoverControllerModeService;
    private ServiceClient<WayPointRequest, WayPointResponse> setHoverControllerWayPointService;
    private Log log;

    public RosServiceProvider(ConnectedNode node) throws ServiceNotFoundException {
        this.log = node.getLog();

        armingService = node.newServiceClient("mavros/cmd/arming", CommandBool._TYPE);
        takeoffService = node.newServiceClient("mavros/cmd/takeoff",CommandTOL._TYPE);
        landService = node.newServiceClient("mavros/cmd/land",CommandTOL._TYPE);
        try {
            setHoverControllerModeService = node.newServiceClient("hovercontroller/switchmode", SwitchMode._TYPE);
            setHoverControllerWayPointService = node.newServiceClient("hovercontroller/setnewwaypoint", hover_controller_msgs.WayPoint._TYPE);
        }
        catch(Exception e){
            log.debug("hover controller related services not exist.");
        }
        setFCUModeService = node.newServiceClient("mavros/set_mode", SetMode._TYPE);
        paramPullService = node.newServiceClient("mavros/param/pull",ParamPull._TYPE);
    }

    public ServiceClient<CommandBoolRequest, CommandBoolResponse> getArmingService() {
        return armingService;
    }

    public ServiceClient<CommandTOLRequest, CommandTOLResponse> getTakeoffService() {return takeoffService;}

    public ServiceClient<CommandTOLRequest, CommandTOLResponse> getLandService() {return landService;}

    public ServiceClient<SetModeRequest, SetModeResponse> getSetFCUModeService(){
        return setFCUModeService;
    }

    public ServiceClient<SwitchModeRequest, SwitchModeResponse> getSetHoverControllerModeService(){ return setHoverControllerModeService; };

    public ServiceClient<WayPointRequest, WayPointResponse> getSetHoverControllerWayPointService() { return setHoverControllerWayPointService;};

    public ServiceClient<ParamPullRequest, ParamPullResponse> getParamPullService() {return paramPullService;}



    public boolean isConnected(){
        return armingService.isConnected()
                //&& setHoverControllerModeService.isConnected()
                //&& setHoverControllerWayPointService.isConnected()
                && setFCUModeService.isConnected()
                && paramPullService.isConnected()
                && takeoffService.isConnected()
                && landService.isConnected();
    }

}
