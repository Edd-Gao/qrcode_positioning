package com.state_machine.core.providers;

import mavros_msgs.BatteryStatus;
import mavros_msgs.ExtendedState;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import mavros_msgs.State;

public class RosSubscriberProvider {

    private Subscriber<State> stateSubscriber;
    private Subscriber<BatteryStatus> batteryStatusSubscriber;
    private Subscriber<ExtendedState> extendedStateSubscriber;

    public RosSubscriberProvider(ConnectedNode node){
        stateSubscriber = node.newSubscriber("mavros/state", State._TYPE);
        batteryStatusSubscriber = node.newSubscriber("mavros/battery", BatteryStatus._TYPE);
        extendedStateSubscriber = node.newSubscriber("mavros/extended_state", ExtendedState._TYPE);
    }

    public Subscriber<State> getStateSubscriber() { return stateSubscriber; }
    public Subscriber<BatteryStatus> getBatteryStatusSubscriber() { return batteryStatusSubscriber;  }
    public Subscriber<ExtendedState> getExtendedStateSubscriber() { return extendedStateSubscriber; }
}
