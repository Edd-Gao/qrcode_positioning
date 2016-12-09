package com.state_machine.core;

import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.providers.*;
import com.state_machine.core.stateMachine.StateMachine;
import com.state_machine.core.stateMachine.utils.StateControlInterface;
import com.state_machine.core.states.State;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.Duration;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import java.util.Queue;

public class StateMachineNode extends AbstractNodeMain {

    private ConnectedNode node;
    private Log log;
    private StateMachine stateMachine;
    private DroneStateTracker droneStateTracker;
    private ActionProvider actionProvider;
    private StateProvider stateProvider;
    private FileProvider fileProvider;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("state_machine");
    }

    @Override
    public void onStart(final ConnectedNode node) {


        this.node = node;
        this.log = node.getLog();
        try {
            Thread.sleep(30000); //forcing 5 seconds wait.

            RosServiceProvider serviceProvider = new RosServiceProvider(node);
            RosSubscriberProvider subscriberProvider = new RosSubscriberProvider(node);
            RosPublisherProvider publisherProvider = new RosPublisherProvider(node);
            RosServerProvider serverProvider = new RosServerProvider(node);
            RosParamProvider rosParamProvider = new RosParamProvider(node,serviceProvider,log);
            Duration timeOut = new Duration(60,0);//todo: remove the magic number here

            droneStateTracker = new DroneStateTracker(
                    subscriberProvider.getStateSubscriber(),
                    subscriberProvider.getBatteryStatusSubscriber(),
                    subscriberProvider.getExtendedStateSubscriber(),
                    subscriberProvider.getLocalPositionPoseSubscriber(),
                    subscriberProvider.getVisionPositionPoseSubscriber(),
                    subscriberProvider.getGlobalPositionGlobalSubscriber()
            );
            actionProvider = new ActionProvider(log, serviceProvider, droneStateTracker, fileProvider, publisherProvider,timeOut,serverProvider );
            stateProvider = new StateProvider(actionProvider, serviceProvider, publisherProvider, log, droneStateTracker);
            fileProvider = new FileProvider(rosParamProvider,actionProvider, stateProvider, log);
            //stateQueue = fileProvider.readScript("/home/firefly/catkin_ws/src/onboard_statemachine/flight_script/test_flight.json");
            //stateQueue = fileProvider.readScript(rosParamProvider.getFlightScriptPath());
            if(!serviceProvider.isConnected()) {
                throw new Exception("service not connected, please run mavros first");
            }

            Thread.sleep(10000);



        State initialState = stateProvider.getIdleState(0);

        stateMachine = new StateMachine(
                initialState,
                stateProvider.getEmergencyLandingState(),
                log,
                droneStateTracker
        );

        final StateControlInterface stateControlInterface = new StateControlInterface(node,stateProvider,stateMachine,rosParamProvider,fileProvider);
        stateControlInterface.wrapHandleRequest("start");

        node.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                if(stateControlInterface.getStateQueue() != null
                   && !stateControlInterface.getStateQueue().isEmpty()
                   && stateMachine.getCurrentState().isIdling()
                   && stateMachine.getCurrentState().isSafeToExit()){
                    stateMachine.setState(stateControlInterface.getStateQueue().remove());
                }
                stateMachine.update(node.getCurrentTime());
                Thread.sleep(20);
            }
        });
        } catch(Exception e) {
            log.fatal("Initialization failed", e);
            System.exit(1);
        }
    }
}
