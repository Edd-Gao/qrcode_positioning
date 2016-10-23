package com.state_machine.core.providers;

import geometry_msgs.Pose;
import mavros_msgs.OverrideRCIn;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import geometry_msgs.PoseStamped;

/**
 * Created by parallels on 5/17/16.
 */
public class RosPublisherProvider {
    private Publisher<OverrideRCIn> overrideRCInPublisher;
    private Publisher<PoseStamped> setpointPositionLocalPublisher;

    public RosPublisherProvider(ConnectedNode node){
        overrideRCInPublisher = node.newPublisher("mavros/rc/override",OverrideRCIn._TYPE);
        setpointPositionLocalPublisher = node.newPublisher("mavros/setpoint_position/local", PoseStamped._TYPE);
    }

    public Publisher<OverrideRCIn> getOverrideRCInPubliser(){
        return overrideRCInPublisher;
    }

    public Publisher<PoseStamped> getSetpointPositionLocalPublisher(){ return setpointPositionLocalPublisher; }

}
