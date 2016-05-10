package com.state_machine.core.node;

import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.providers.*;
import com.state_machine.core.stateMachine.StateMachine;
import com.state_machine.core.states.State;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

import java.util.List;

public class StateMachineNode extends AbstractNodeMain {

    private ConnectedNode node;
    private Log log;
    private StateMachine stateMachine;
    private DroneStateTracker droneStateTracker;
    private ActionProvider actionProvider;
    private StateProvider stateProvider;
    private FileProvider fileProvider;
    private List<State> script;

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
            droneStateTracker = new DroneStateTracker(
                    subscriberProvider.getStateSubscriber(),
                    subscriberProvider.getBatteryStatusSubscriber(),
                    subscriberProvider.getExtendedStateSubscriber()
            );
            actionProvider = new ActionProvider(serviceProvider, droneStateTracker);
            stateProvider = new StateProvider(actionProvider, serviceProvider, log);
            fileProvider = new FileProvider(actionProvider, stateProvider, log);
            script = fileProvider.readScript("flightscript.json");
        } catch(Exception e) {
            log.fatal("Initialization failed", e);
            System.exit(1);
        }

        State initialState = stateProvider.getIdleState();
        if(!script.isEmpty()) initialState = script.get(0);

        stateMachine = new StateMachine(
                initialState,
                stateProvider.getShutdownState(),
                stateProvider.getManualControlState(),
                log,
                droneStateTracker
        );

        node.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                stateMachine.update(node.getCurrentTime());
                Thread.sleep(33);
            }
        });
    }
}
