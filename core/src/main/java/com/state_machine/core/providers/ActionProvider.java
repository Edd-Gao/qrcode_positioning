package com.state_machine.core.providers;

import com.state_machine.core.actions.*;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.actions.util.Waypoint;
import com.state_machine.core.droneState.NeighborStateTracker;
import org.apache.commons.logging.Log;
import org.ros.message.Duration;

public class ActionProvider implements FlyToActionFactory {

    private RosServiceProvider serviceProvider;
    private DroneStateTracker stateTracker;
    private NeighborStateTracker neighborStateTracker;
    private RosPublisherProvider rosPublisherProvider;
    private ArmAction armAction;
    private DisarmAction disarmAction;
    private LandingAction landingAction;
    private PX4LandAction px4LandAction;
    private DecentralizedAction decentralizedAction;
    private FileProvider fileProvider;
    private RosServerProvider rosServerProvider;
    private Duration timeOut;
    private Log logger;

    public ActionProvider(
            Log logger,
            RosServiceProvider serviceProvider,
            DroneStateTracker stateTracker,
            NeighborStateTracker neighborStateTracker,
            FileProvider fileProvider,
            RosPublisherProvider rosPublisherProvider,
            Duration timeOut,
            RosServerProvider rosServerProvider,
            RosParamProvider rosParamProvider){
        this.serviceProvider = serviceProvider;
        this.stateTracker = stateTracker;
        this.neighborStateTracker = neighborStateTracker;
        this.fileProvider = fileProvider;
        this.rosPublisherProvider = rosPublisherProvider;
        this.rosServerProvider = rosServerProvider;
        this.timeOut = timeOut;
        this.logger = logger;
        armAction = new ArmAction(logger, serviceProvider.getArmingService(), stateTracker);
        disarmAction = new DisarmAction(serviceProvider.getArmingService(), stateTracker);
        landingAction = new LandingAction(serviceProvider.getSetHoverControllerModeService(), stateTracker,rosServerProvider,timeOut);
        px4LandAction = new PX4LandAction(stateTracker,timeOut,serviceProvider.getLandService());
        decentralizedAction = new DecentralizedAction(logger,stateTracker,neighborStateTracker,timeOut, rosParamProvider, , rosServiceProvider);
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

    public PX4FlyToAction getPX4FlyToAction(Waypoint objective,int seq){
        return new PX4FlyToAction(logger,objective,stateTracker,rosPublisherProvider,timeOut,serviceProvider,seq);
    }

    public SetFCUModeAction getSetFCUModeAction(String newMode){
        return new SetFCUModeAction(serviceProvider.getSetFCUModeService(),newMode);
    }

    public PX4LandAction getPX4LandAction(){ return px4LandAction;}

    public PX4TakeoffAction getPX4TakeoffAction(double target_heightm){
        return new PX4TakeoffAction(logger, stateTracker,target_heightm,timeOut,serviceProvider.getTakeoffService());
    }

    public DecentralizedAction getDecentralizedAction(){return decentralizedAction;}
}
