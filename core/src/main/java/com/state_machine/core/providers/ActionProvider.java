package com.state_machine.core.providers;

import com.state_machine.core.actions.*;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.actions.util.Waypoint;

public class ActionProvider implements FlyToActionFactory {

    private RosServiceProvider serviceProvider;
    private DroneStateTracker stateTracker;
    private RosPublisherProvider rosPublisherProvider;
    private ArmAction armAction;
    private DisarmAction disarmAction;
    private LandingAction landingAction;
    private TakeoffAction takeoffAction;
    private FileProvider fileProvider;

    public ActionProvider(
            RosServiceProvider serviceProvider,
            DroneStateTracker stateTracker,
            FileProvider fileProvider,
            RosPublisherProvider rosPublisherProvider
    ){
        this.serviceProvider = serviceProvider;
        this.stateTracker = stateTracker;
        this.fileProvider = fileProvider;
        this.rosPublisherProvider = rosPublisherProvider;
        armAction = new ArmAction(serviceProvider.getArmingService(), stateTracker, fileProvider, rosPublisherProvider);
        disarmAction = new DisarmAction(serviceProvider.getArmingService(), stateTracker);
        takeoffAction = new TakeoffAction(serviceProvider.getTakeoffService(), stateTracker);
        landingAction = new LandingAction(serviceProvider.getLandingService(), stateTracker);
    }

    public ArmAction getArmAction(){ return armAction; }
    public DisarmAction getDisarmAction(){ return disarmAction; }
    public TakeoffAction getTakeoffAction(){ return takeoffAction; }
    public LandingAction getLandingAction(){ return landingAction; }

    public FlyToAction getFlyToAction(Waypoint objective){
        return new FlyToAction(objective, stateTracker, fileProvider);
    }
}
