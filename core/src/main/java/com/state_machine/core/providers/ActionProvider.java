package com.state_machine.core.providers;

import com.state_machine.core.actions.*;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.actions.util.Waypoint;

public class ActionProvider implements FlyToActionFactory {

    private RosServiceProvider serviceProvider;
    private DroneStateTracker stateTracker;
    private ArmAction armAction;
    private DisarmAction disarmAction;
    private LandingAction landingAction;
    private TakeoffAction takeoffAction;

    public ActionProvider(
            RosServiceProvider serviceProvider,
            DroneStateTracker stateTracker
    ){
        this.serviceProvider = serviceProvider;
        this.stateTracker = stateTracker;
        armAction = new ArmAction(serviceProvider.getArmingService(), stateTracker);
        disarmAction = new DisarmAction(serviceProvider.getArmingService(), stateTracker);
        takeoffAction = new TakeoffAction(serviceProvider.getTakeoffService(), stateTracker);
        landingAction = new LandingAction(serviceProvider.getLandingService(), stateTracker);
    }

    public ArmAction getArmAction(){ return armAction; }
    public DisarmAction getDisarmAction(){ return disarmAction; }
    public TakeoffAction getTakeoffAction(){ return takeoffAction; }
    public LandingAction getLandingAction(){ return landingAction; }

    public FlyToAction getFlyToAction(Waypoint objective){
        return new FlyToAction(objective, stateTracker);
    }
}
