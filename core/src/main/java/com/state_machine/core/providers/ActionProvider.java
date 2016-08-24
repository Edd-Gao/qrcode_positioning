package com.state_machine.core.providers;

import com.state_machine.core.actions.*;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.actions.util.Waypoint;
import org.ros.message.Duration;

public class ActionProvider implements FlyToActionFactory {

    private RosServiceProvider serviceProvider;
    private DroneStateTracker stateTracker;
    private RosPublisherProvider rosPublisherProvider;
    private ArmAction armAction;
    private DisarmAction disarmAction;
    private LandingAction landingAction;
    private FileProvider fileProvider;
    private RosServerProvider rosServerProvider;
    private Duration timeOut;

    public ActionProvider(
            RosServiceProvider serviceProvider,
            DroneStateTracker stateTracker,
            FileProvider fileProvider,
            RosPublisherProvider rosPublisherProvider,
            Duration timeOut,
            RosServerProvider rosServerProvider){
        this.serviceProvider = serviceProvider;
        this.stateTracker = stateTracker;
        this.fileProvider = fileProvider;
        this.rosPublisherProvider = rosPublisherProvider;
        this.rosServerProvider = rosServerProvider;
        this.timeOut = timeOut;
        armAction = new ArmAction(serviceProvider.getArmingService(), stateTracker);
        disarmAction = new DisarmAction(serviceProvider.getArmingService(), stateTracker);
        landingAction = new LandingAction(serviceProvider.getSetHoverControllerModeService(), stateTracker,rosServerProvider,timeOut);
    }

    public ArmAction getArmAction(){ return armAction; }
    public DisarmAction getDisarmAction(){ return disarmAction; }
    public LandingAction getLandingAction(){ return landingAction; }

    public TakeoffAction getTakeoffAction(float target_heightcm){
        return new TakeoffAction(serviceProvider.getSetHoverControllerModeService(),stateTracker,target_heightcm,rosServerProvider,timeOut);
    }

    public FlyToAction getFlyToAction(Waypoint objective){
        return new FlyToAction(objective, stateTracker, fileProvider,rosServerProvider,serviceProvider.getSetHoverControllerModeService(),serviceProvider.getSetHoverControllerWayPointService() ,timeOut );
    }

    public SetFCUModeAction getSetFCUModeAction(String newMode){
        return new SetFCUModeAction(serviceProvider.getSetFCUModeService(),newMode);
    }
}
