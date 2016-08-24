package com.state_machine.core;

import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.providers.*;
import com.state_machine.core.stateMachine.StateMachine;
import com.state_machine.core.states.State;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
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
    private Queue<State> stateQueue;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("state_machine");
    }

    @Override
    public void onStart(final ConnectedNode node) {
        this.node = node;
        this.log = node.getLog();
        try {

            RosServiceProvider serviceProvider = new RosServiceProvider(node);
            RosSubscriberProvider subscriberProvider = new RosSubscriberProvider(node);
            RosPublisherProvider publisherProvider = new RosPublisherProvider(node);
            RosServerProvider serverProvider = new RosServerProvider(node);
            Duration timeOut = new Duration(10,0);//todo: remove the magic number here

            droneStateTracker = new DroneStateTracker(
                    subscriberProvider.getStateSubscriber(),
                    subscriberProvider.getBatteryStatusSubscriber(),
                    subscriberProvider.getExtendedStateSubscriber()
            );
            actionProvider = new ActionProvider(serviceProvider, droneStateTracker, fileProvider, publisherProvider,timeOut,serverProvider );
            stateProvider = new StateProvider(actionProvider, serviceProvider, publisherProvider, log);
            fileProvider = new FileProvider(actionProvider, stateProvider, log);
            stateQueue = fileProvider.readScript("flightscript.json");

        } catch(Exception e) {
            log.fatal("Initialization failed", e);
            System.exit(1);
        }

        State initialState = stateProvider.getIdleState();
        if(!stateQueue.isEmpty()) initialState = stateQueue.remove();

        stateMachine = new StateMachine(
                initialState,
                stateProvider.getEmergencyLandingState(),
                log,
                droneStateTracker
        );

        node.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                if(!stateQueue.isEmpty()
                   && stateMachine.getCurrentState().isIdling()
                   && stateMachine.getCurrentState().isSafeToExit()){
                    stateMachine.setState(stateQueue.remove());
                }
                stateMachine.update(node.getCurrentTime());
                Thread.sleep(33);
            }
        });
    }
}
