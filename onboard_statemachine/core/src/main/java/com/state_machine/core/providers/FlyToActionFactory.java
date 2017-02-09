package com.state_machine.core.providers;

import com.state_machine.core.actions.FlyToAction;
import com.state_machine.core.actions.util.Waypoint;

public interface FlyToActionFactory {

    FlyToAction getFlyToAction(Waypoint objective);
}
