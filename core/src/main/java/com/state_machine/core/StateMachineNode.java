package com.state_machine.core;

import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import std_msgs.*;
import std_msgs.String;

public class StateMachineNode extends AbstractNodeMain {

    private StateMachine stateMachine;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rosjava/listener");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        stateMachine = new StateMachine();
        final Log log = connectedNode.getLog();

        Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("chatter", std_msgs.String._TYPE);
        subscriber.addMessageListener(new MessageListener<String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                log.info("I heard: \"" + message.getData() + "\"");

            }
        });

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            //private int sequenceNumber;

            @Override
            protected void setup() {
                //sequenceNumber = 0;

            }

            @Override
            protected void loop() throws InterruptedException {

                //for reference
                //std_msgs.String str = publisher.newMessage();
                //str.setData("Hello world! " + sequenceNumber);
                //publisher.publish(str);



                //sequenceNumber++;
                Thread.sleep(1000);

            }
        });


    }
}
