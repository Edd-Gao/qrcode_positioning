# Actions #

## Types of actions ##
- ArmAction/DisarmAction: Arms/disarms the drone.
- TakeoffAction/LandingAction: Takes off or lands the drone.
- FlyToAction: Attempts to fly to the given coordinates.
- SetFCUModeAction: set the mode of FCU
- PX4TakeoffAction/ PX4LandAction: takes off or lands the drone with mavros services. the implementation is in PX4 firmware.
- PX4FlyToAction: Attempts to fly to the given coordinates with PX4 firmware implementation.
## Structure ##

> constructor
Nearly every action requires as contructor parameters at least a DroneStateTracker object and a ServiceClient for the service they need to call to execute the action.

> public void enterAction()
Will be executed every time some state selects this action to be executed. Should at least reset the state of the action.

> public void exitAction()
Will be executed every time some state chooses another action to be executed and this one has been active beforehand.

> public ActionStatus loopAction(Time time)
This is called every update cycle when this is the currently executed action. It returns the execution status of the action (Waiting, Success, ConnectionFailure, Failure, Inactive) during that cycle. If there's a way to determine from the current drone state that the action does not need to be executed or that there's a connection error, the method should simply return Success or ConnectionError, respectively. Otherwise, if no service message has been sent yet, the action will send a service message with a ServiceResponseListener that changes the status of the action once a response message is received. If a response has been received during the last cycle, the method will return Success or Error, otherwise it returns Waiting.

## Adding a new Action ##

1. Determine which service(s) the action needs to call to perform itself. Implement those services in RosServiceProvider.java.
2. Determine what information the action needs to know whether it is done (Service response messages usually work, but may not always be the best option). Implement subscribers for that information in RosSubscriberProvider.java. Add them to the constructor of DroneStateTracker.java and create MessageListeners for them. Implement a getter method to DroneStateTracker to query the most up-to-date information received by the MessageListeners.
3. Add a DroneStateTracker and your new services as constructor parameters of your new action. Implement the loopAction and toString methods of your action.
4. Create your new action in ActionProvider.java. It includes all the parameters your action needs to be constructed.
