package com.state_machine.core.providers;

import mavros_msgs.OverrideRCIn;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

/**
 * Created by parallels on 5/17/16.
 */
public class RosPublisherProvider {
    private Publisher<OverrideRCIn> overrideRCInPublisher;

    public RosPublisherProvider(ConnectedNode node){
        overrideRCInPublisher = node.newPublisher("mavros/rc/override",OverrideRCIn._TYPE);
    }

    public Publisher<OverrideRCIn> getOverrideRCInPubliser(){
        return overrideRCInPublisher;
    }


}
