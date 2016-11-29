package com.state_machine.core.droneState;

import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovariance;
import mavros_msgs.ExtendedState;
import mavros_msgs.State;
import mavros_msgs.BatteryStatus;
import nav_msgs.Odometry;
import org.ros.internal.message.DefaultMessageFactory;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.NavSatFix;

public class DroneStateTracker {

    private boolean armed;      //the drone arming status
    private float battery;              //battery remaining
    private DroneLanded droneLanded;    //the drone landing status
    private double[] localPosition = new double[3];
    private double lattitude;
    private double longitude;
    private double altitude;
    private String FCUMode;

    public DroneStateTracker(
            Subscriber<State> stateSubscriber,
            Subscriber<BatteryStatus> batterySubscriber,
            Subscriber<ExtendedState> extendedStateSubscriber,
            final Subscriber<PoseStamped> localPositionPoseSubscriber,
            Subscriber<NavSatFix> globalPositionGlobalSubscriber
            ){
        MessageListener<NavSatFix> globalPositionListener = new MessageListener<NavSatFix>() {
            @Override
            public void onNewMessage(NavSatFix navSatFix) {
                lattitude = navSatFix.getLatitude();
                longitude = navSatFix.getLongitude();
                altitude = navSatFix.getAltitude();
            }
        };
        globalPositionGlobalSubscriber.addMessageListener(globalPositionListener);

        MessageListener<PoseStamped> localPoseListener = new MessageListener<PoseStamped>() {
            @Override
            public void onNewMessage(PoseStamped poseStamped) {
                localPosition[0] = poseStamped.getPose().getPosition().getX();
                localPosition[1] = poseStamped.getPose().getPosition().getY();
                localPosition[2] = poseStamped.getPose().getPosition().getZ();
            }
        };
        localPositionPoseSubscriber.addMessageListener(localPoseListener);

        MessageListener<State> stateListener = new MessageListener<State>() {
            @Override
            public void onNewMessage(State state) {
                armed = state.getArmed();
                FCUMode = state.getMode();
            }
        };
        stateSubscriber.addMessageListener(stateListener);

        MessageListener<BatteryStatus> batteryListener = new MessageListener<BatteryStatus>() {
            @Override
            public void onNewMessage(BatteryStatus status) {
                battery = status.getRemaining();
            }
        };
        batterySubscriber.addMessageListener(batteryListener);

        MessageListener<ExtendedState> extendedStateListener = new MessageListener<ExtendedState>() {
            @Override
            public void onNewMessage(ExtendedState extendedState) {
                switch(extendedState.getLandedState()){
                    case 0: droneLanded = DroneLanded.Undefined;
                        break;
                    case 1: droneLanded = DroneLanded.OnGround;
                        break;
                    case 2: droneLanded = DroneLanded.InAir;
                        break;
                }
            }
        };
        extendedStateSubscriber.addMessageListener(extendedStateListener);

        armed = false;
        battery = -1;
        droneLanded = DroneLanded.Undefined;
    }

    public boolean getArmed(){
        return armed;
    }
    public float getRemaining() {return battery;}
    public DroneLanded getDroneLanded() {return droneLanded;}
    public double[] getLocalPosition() { return localPosition;}
    public double getLattitude() { return lattitude;}
    public double getLongitude() { return longitude;}
    public double getAltitude() { return  altitude;}
    public String getFCUMode() { return FCUMode;}

    public boolean initialized(){
        if (battery == -1 || droneLanded == DroneLanded.Undefined)
            return false;
        else
            return true;
    }
}
