# States #

## Types of states ##
- IdleState: switch to OFFBOARD mode and stay for 3 seconds, then it is safe to exit.
- ScriptedState:  switch to OFFBOARD mode and execute a list of scripted actions.
- EmergencyLandingState: switch to OFFBOARD mode and land at current position.

## Structure ##

> constructor
Nearly every state requires an ActionProvider to create selected action and a ServiceClient to set the mode the state is associated with.

> public void enterAction()
Will be executed every time this state is set inside the StateMachine. Should at least reset the state of the state.

> public void exitAction()
Will be executed every time this state has been the active state of the StateMachine but another state is selected. Should call the exitAction of the current action, if it exists.

> public void update()
This is called every update cycle when this state is the active state. If a current action is selected, it updates that action and checks its status with the handleActionResult method. If no current action is selected, it calls the chooseNextAction method.

> protected abstract void chooseNextAction(Time time)
Creates a new action and sets it as the current action of the state.

> private boolean handleActionResult(ActionStatus status)
If the status implies successful execution, the method sets as current action the next action from the prerequisite list, if any, and otherwise sets it as null. If the status implies failure, the method saves the information about the failure for the state machine to check.

> public abstract void isSafeToExit()
Returns true if exiting this state at this moment won't put the drone in danger. (For example, if the drone was executing some agile maneuver, a sudden state switch could lead to a situation where the new state could not return the drone back into a stable position.)

> public abstract boolean isIdling()
Returns true if this state is not doing anything meaningful anymore and any other state can be prioritized over it.

## Adding a new State ##

1. Create all the Actions the State will be using and add them to ActionProvider.java. If your state also needs a new ServiceClient, add it to RosServiceProvider.java and the constructor of your state.
2. Define chooseNextAction, isIdling and isSafeToExit. Overriding enterAction or exitAction can also be done, but the overriding methods should call super.overriddenMethod().
3. Add a factory to StateProvider.java that can create your new state. StateProvider.java contains all the dependencies of your state, all you need to do is put them into your state's constructor parameters.
