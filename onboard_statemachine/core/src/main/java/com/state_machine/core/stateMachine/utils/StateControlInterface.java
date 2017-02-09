package com.state_machine.core.stateMachine.utils;

import com.state_machine.core.actions.util.*;
import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.providers.FileProvider;
import com.state_machine.core.providers.RosParamProvider;
import com.state_machine.core.providers.StateProvider;
import com.state_machine.core.stateMachine.StateMachine;
import com.state_machine.core.states.IdleState;
import com.state_machine.core.states.State;
import org.apache.commons.logging.Log;
import org.ros.node.ConnectedNode;
import onboard_statemachine_msgs.*;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import com.state_machine.core.states.IdleState;

import java.io.File;
import java.util.Queue;

/**
 * Created by firefly on 12/9/16.
 */
public class StateControlInterface {
    private Queue<State> stateQueue;
    private FileProvider fileProvider;
    private RosParamProvider rosParamProvider;
    private StateMachine stateMachine;
    private StateProvider stateProvider;
    private Log log;

    public StateControlInterface(ConnectedNode node,
                                 StateProvider stateProvider,
                                 StateMachine stateMachine,
                                 RosParamProvider rosParamProvider,
                                 FileProvider fileProvider){
        this.rosParamProvider = rosParamProvider;
        this.stateProvider = stateProvider;
        this.stateMachine = stateMachine;
        this.fileProvider = fileProvider;
        this.log = node.getLog();
        node.newServiceServer("~ControlInterface", StateMachineControl._TYPE,
                new ServiceResponseBuilder<StateMachineControlRequest, StateMachineControlResponse>() {
                    @Override
                    public void build(StateMachineControlRequest request, StateMachineControlResponse response){
                        response.setSuccess(handleRequest(request.getOperation()));
                    }
                });
    }

    //wrapper of handleRequest for other modules
    public boolean wrapHandleRequest(String request){
        return handleRequest(request);
    }

    private boolean handleRequest(String request){
        if(request.equals("start")){
            stateQueue = fileProvider.readScript(rosParamProvider.getFlightScriptPath());
            stateMachine.setStatemachineRunningFlag(true);
        }else if(request.equals("stop")){

            if(stateMachine.getCurrentState().isSafeToExit()){
                stateMachine.setStatemachineRunningFlag(false);
                while(!stateQueue.isEmpty()) {
                    stateQueue.remove();
                }
                stateMachine.setState(stateProvider.getIdleState(1000));
            }
        }else if(request.equals("restart")){
            if(stateMachine.getCurrentState().isSafeToExit()){
                stateMachine.setStatemachineRunningFlag(false);
                while(!stateQueue.isEmpty()) {
                    stateQueue.remove();
                }
                stateMachine.setState(stateProvider.getIdleState(1000));
            }
            stateQueue = fileProvider.readScript(rosParamProvider.getFlightScriptPath());
            stateMachine.setStatemachineRunningFlag(true);
        }else if(request.equals("pause")){
            stateMachine.setStatemachineRunningFlag(false);
        }else if(request.equals("resume")){
            stateMachine.setStatemachineRunningFlag(true);
        }else{
            log.warn("unsupported request.supported requests are 'start', 'stop', 'restart', 'pause', 'resume'.");
            return false;
        }

        return true;
    }

    public    Queue<State> getStateQueue(){return stateQueue;}


}
