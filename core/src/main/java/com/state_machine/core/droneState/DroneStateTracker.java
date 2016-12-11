package com.state_machine.core.droneState;

import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovariance;
import geometry_msgs.TwistStamped;
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
    private double[] localVelocity = new double[3];
    private double[] localPosition = new double[3];
    private double[] localOrigin = new double[3];// temporally use this variable to store the local origin coordinate in world frame.
    private double[] visionPosition = new double[3];
    private double lattitude;
    private double longitude;
    private double altitude;
    private String FCUMode;

    public DroneStateTracker(
            Subscriber<State> stateSubscriber,
            Subscriber<BatteryStatus> batterySubscriber,
            Subscriber<ExtendedState> extendedStateSubscriber,
            Subscriber<TwistStamped> localPositionVelocitySubscriber,
            Subscriber<PoseStamped> localPositionPoseSubscriber,
            Subscriber<PoseStamped> visionPositionPoseSubscriber,
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

        MessageListener<TwistStamped> localVelocityListener = new MessageListener<TwistStamped>() {
            @Override
            public void onNewMessage(TwistStamped twistStamped) {
                localVelocity[0] = twistStamped.getTwist().getLinear().getX();
                localVelocity[1] = twistStamped.getTwist().getLinear().getY();
                localVelocity[2] = twistStamped.getTwist().getLinear().getZ();
            }
        };
        localPositionVelocitySubscriber.addMessageListener(localVelocityListener);

        MessageListener<PoseStamped> localPoseListener = new MessageListener<PoseStamped>() {
            @Override
            public void onNewMessage(PoseStamped poseStamped) {
                localPosition[0] = poseStamped.getPose().getPosition().getX();
                localPosition[1] = poseStamped.getPose().getPosition().getY();
                localPosition[2] = poseStamped.getPose().getPosition().getZ();
            }
        };
        localPositionPoseSubscriber.addMessageListener(localPoseListener);

        MessageListener<PoseStamped> visionPoseListener = new MessageListener<PoseStamped>() {
            @Override
            public void onNewMessage(PoseStamped poseStamped) {
                visionPosition[0] = poseStamped.getPose().getPosition().getX();
                visionPosition[1] = poseStamped.getPose().getPosition().getY();
                visionPosition[2] = poseStamped.getPose().getPosition().getZ();
            }
        };
        visionPositionPoseSubscriber.addMessageListener(visionPoseListener);

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

    //getters
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
    public double[] getLocalOrigin(){return localOrigin;}
    public double[] getVisionPosition(){return visionPosition; }
    public double[] getLocalVelocity(){return localVelocity;}

    //setters
    public void setLocalOrigin(double[] originValue){
        localOrigin[0] = originValue[0];
        localOrigin[1] = originValue[1];
        localOrigin[2] = originValue[2];
    }

    public boolean initialized(){
        if (battery == -1 || droneLanded == DroneLanded.Undefined)
            return false;
        else
            return true;
    }
}
