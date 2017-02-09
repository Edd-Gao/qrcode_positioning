# State Machine #

The state machine (StateMachine.java) defines the active state of the program. When StateMachine.update() is called, the current state inside the state machine and no other state is polled for updates.

## Structure ##

> constructor
The StateMachine requires an initial state, an emergency landing state, a manual control state and a DroneStateTracker.

> public void update(Time time)
This method is called by the StateMachineNode on every update cycle. It will query the failsafeCheck method for possible errors, and if there's no error, poll the current state for updates. If there is an error, the error is logged and the current state is reflexively switched to either manual control or emergency landing depending on the nature of the error.

> private ErrorType failsafeCheck()
Checks drone state information such as remaining battery and any failures logged by the current state. If no error is found, returns ErrorType.NoError.

> public void setState(State newState)
If current state is safe to exit, switches to newState. Calls exitAction for currentState and enterAction for newState. If current state is not safe to exit, throws a StateException.

> public void forceState(State newState)
Same as setState, except it does not check for safety. Should only be used when the drone is in an unsafe state already. For example, the StateMachine error control state switches use forceState.

## Using the state machine ##

The state machine is initialized in StateMachineNode.java. The initial state is set during the construction of the state machine.

There are two ways to switch to a new state. One is to add the new state into the state queue of the node. The state will then be selected after every state before it has been selected and later become inactive. The other way is to call stateMachine.setState(newState), which instantly sets the new state as active, as long as it's safe to do so.
