package com.state_machine.core.providers;

import geometry_msgs.PoseStamped;
import mavros_msgs.BatteryStatus;
import mavros_msgs.ExtendedState;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import mavros_msgs.State;
import sensor_msgs.NavSatFix;
import nav_msgs.Odometry;

public class RosSubscriberProvider {

    private Subscriber<State> stateSubscriber;
    private Subscriber<BatteryStatus> batteryStatusSubscriber;
    private Subscriber<ExtendedState> extendedStateSubscriber;
    private Subscriber<NavSatFix> globalPositionGlobalSubscriber;
    private Subscriber<PoseStamped> localPositionPoseSubscriber;

    public RosSubscriberProvider(ConnectedNode node){
        stateSubscriber = node.newSubscriber("mavros/state", State._TYPE);
        batteryStatusSubscriber = node.newSubscriber("mavros/battery", BatteryStatus._TYPE);
        extendedStateSubscriber = node.newSubscriber("mavros/extended_state", ExtendedState._TYPE);
        globalPositionGlobalSubscriber = node.newSubscriber("mavros/global_position/global", NavSatFix._TYPE);
        localPositionPoseSubscriber = node.newSubscriber("mavros/vision_pose/pose", PoseStamped._TYPE);
    }

    public Subscriber<State> getStateSubscriber() { return stateSubscriber; }
    public Subscriber<BatteryStatus> getBatteryStatusSubscriber() { return batteryStatusSubscriber;  }
    public Subscriber<ExtendedState> getExtendedStateSubscriber() { return extendedStateSubscriber; }
    public Subscriber<NavSatFix> getGlobalPositionGlobalSubscriber() { return globalPositionGlobalSubscriber;}
    public Subscriber<PoseStamped> getLocalPositionPoseSubscriber() { return localPositionPoseSubscriber;}
}
