package com.state_machine.core.droneState;

import mavros_msgs.ExtendedState;
import mavros_msgs.State;
import mavros_msgs.BatteryStatus;
import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;

public class DroneStateTracker {

    private boolean armed;      //the drone arming status
    private float battery;              //battery remaining
    private DroneLanded droneLanded;    //the drone landing status

    public DroneStateTracker(
            Subscriber<State> stateSubscriber,
            Subscriber<BatteryStatus> batterySubscriber,
            Subscriber<ExtendedState> extendedStateSubscriber
    ){
        MessageListener<State> stateListener = new MessageListener<State>() {
            @Override
            public void onNewMessage(State state) {
                armed = state.getArmed();
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

    public boolean initialized(){
        if (battery == -1 || droneLanded == DroneLanded.Undefined)
            return false;
        else
            return true;
    }
}
