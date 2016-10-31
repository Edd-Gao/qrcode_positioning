# Drone state tracking #

## General ##

ROS and ROSJava provide a message-based system to query information about the state of the drone. ROS nodes that contain information, provide messages to Topics that other node can subscribe to. This is done by creating a Subscriber to access that Topic and adding a MessageListener object to the Subscriber.

## DroneStateTracker.java ##

This class is essentially a collection of subscriptions to various topics. It hides the complexity of the asynchronous process and low-level message content behind a set of getter methods that give their callers the most up-to-date information in a Java-friendly package. It receives all the required subscribers as constructor parameters, creates MessageListeners for them, picks out the needed information from the received messages and translates it to Java objects.

Generally, every object that needs to know things such as the arming state or battery level of the drone should be given a DroneStateTracker as a constructor parameter.

## Adding a new subscriber to DroneStateTracker.java ##

1. Create the subscriber in RosSubscriberProvider.java.
2. Add the new subscriber to the constructor of DroneStateTracker.java.
3. Inside DroneStateTracker, create a MessageListener for the new Subscriber and have the onNewMessage method of the MessageListener save the received information inside a private field of the DroneStateTracker.
4. Create a getter method for the field that the onNewMessage method mutates.
