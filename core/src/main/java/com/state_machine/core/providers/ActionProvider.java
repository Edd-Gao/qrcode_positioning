package com.state_machine.core.providers;

import com.state_machine.core.actions.ArmAction;
import com.state_machine.core.actions.DisarmAction;
import com.state_machine.core.actions.LandingAction;
import com.state_machine.core.actions.TakeoffAction;
import com.state_machine.core.droneState.DroneStateTracker;

public class ActionProvider {

    private ArmAction armAction;
    private DisarmAction disarmAction;
    private LandingAction landingAction;
    private TakeoffAction takeoffAction;

    public ActionProvider(
            RosServiceProvider serviceProvider,
            DroneStateTracker stateTracker
    ){
        armAction = new ArmAction(serviceProvider.getArmingService(), stateTracker);
        disarmAction = new DisarmAction(serviceProvider.getArmingService(), stateTracker);
        takeoffAction = new TakeoffAction(serviceProvider.getTakeoffService(), stateTracker);
        landingAction = new LandingAction(serviceProvider.getLandingService(), stateTracker);
    }

    public ArmAction getArmAction(){ return armAction; }
    public DisarmAction getDisarmAction(){ return disarmAction; }
    public TakeoffAction getTakeoffAction(){ return takeoffAction; }
    public LandingAction getLandingAction(){ return landingAction; }
}
