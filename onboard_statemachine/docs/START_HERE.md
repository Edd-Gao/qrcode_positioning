# The State Machine Node #

This module's purpose is to receive messages about how the drone should act and either translate those to concrete actions or forward them to a module that can make the choice about what concrete actions to undertake.

## Core program logic ##

The core of the program execution can be found at StateMachineNode.java. It extends AbstractNodeMain, which essentially contains the main method of a RosJava program. The method onStart is triggered as the node is started. onStart initializes the program data and begins an infinite loop that polls changes in the state queue of the node, sets new states and controls the update pace of the state machine itself. When this infinite loop is canceled by an exception, the node shuts down.

Three ways to change state are provided:
+ A state queue, which the node polls after it notices that the current state is only performing actions that signify idleness.
+ Best-effort instant state switching: The state is changed instantly if doing so won't set the drone in danger. If it would do so, the new state is instead moved to the front of the state queue.
+ Forced state switching: The state is changed instantly no matter what. This should only be done if the drone is already in danger and the current state cannot be trusted to handle it.

*Note: StateMachineNode currently does not initialize any message listeners, but it should implement the ROS service server interface of the node in the future. This service will receive messages about which state the node should be set to.*

## Program structure ##

### State Machine ###

StateMachine.java keeps track of the current state and the general status information of the drone. States should only ever be used through the state machine.

Generally, the state machine passively waits for commands to set a new state while updating the current state, but it also contains the error handling logic of the program and has the authority to make emergency decisions when an error is encountered.

More information can be found in stateMachine.md.

### Actions ###

An action contains the information on how to make the drone perform the specified action. It commands the drone through a Mavros interface and listens to indicators of when the action is either performed or an error has occurred.

More information can be found in actions.md.

### States ###

A state contains the logic on which concrete actions to choose and a list of actions that need to be performed before other actions can be chosen to be executed. The state will poll the currently chosen action for its state of execution, and select the next action after the current one tells that it is finished. If the current action informs the state about an error, the state will make that information accessible to the state machine.

More information can be found in states.md.

### Providers ###

The providers are a set of interconnected factories that provide the initialization logic of the program in a dependency injection friendly way. New objects should always be created using a provider, and provider interfaces should be given as constructor parameters to objects that need to create new objects.

More information can be found in providers.md.

### Scripting ###

For testing and similar purposes, the node initialization can be directed using a json script. This script defines the initial content of the state queue that the state machine uses to select new states from.

More information can be found in scripting.md.
