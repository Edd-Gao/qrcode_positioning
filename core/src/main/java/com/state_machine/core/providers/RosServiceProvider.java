package com.state_machine.core.providers;

import com.state_machine.core.actions.ArmAction;
import mavros_msgs.*;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;

public class RosServiceProvider {

    private ServiceClient<CommandBoolRequest, CommandBoolResponse> armingService;
    private ServiceClient<CommandTOLRequest, CommandTOLResponse> takeoffService;
    private ServiceClient<CommandTOLRequest, CommandTOLResponse> landingService;
    private ServiceClient<SetModeRequest, SetModeResponse> setModeService;

    public RosServiceProvider(ConnectedNode node) throws ServiceNotFoundException {
        armingService = node.newServiceClient("mavros/cmd/arming", CommandBool._TYPE);
        takeoffService = node.newServiceClient("mavros/cmd/takeoff", CommandTOL._TYPE);
        landingService = node.newServiceClient("mavros/cmd/land", CommandTOL._TYPE);
        setModeService = node.newServiceClient("mavros/set_mode", SetMode._TYPE);
    }

    public ServiceClient<CommandBoolRequest, CommandBoolResponse> getArmingService() {
        return armingService;
    }

    public ServiceClient<CommandTOLRequest, CommandTOLResponse> getTakeoffService() {
        return takeoffService;
    }

    public ServiceClient<CommandTOLRequest, CommandTOLResponse> getLandingService() {
        return landingService;
    }

    public ServiceClient<SetModeRequest, SetModeResponse> getSetModeService(){
        return setModeService;
    }
}
