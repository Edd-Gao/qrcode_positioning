package com.state_machine.core.droneState;

import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import geometry_msgs.TwistStamped;
import org.apache.commons.logging.Log;
import org.ros.master.client.MasterStateClient;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.namespace.NodeNameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;


import java.util.Vector;

/**
 * Created by Gao Changyu on 12/11/16.
 */
public class NeighborStateTracker {

    private ConnectedNode node;
    private String thisDroneName;
    private MasterStateClient masterStateClient;
    private int neighborNum;
    private Vector<Subscriber<PoseStamped> > neighborVisionPoseSubscribers;
    private Vector<Subscriber<TwistStamped> > neighborLocalVelocitySubscribers;
    private Vector<double[]> neighborVisionPoses;
    private Vector<double[]> neighborLocalVelocities;
    private Log log;


    public NeighborStateTracker(ConnectedNode node){
        this.log = node.getLog();
        this.node = node;
        thisDroneName = node.getName().getParent().getBasename().toString();
        log.info("drone name:" + thisDroneName);
        masterStateClient = new MasterStateClient(node,node.getMasterUri());
        neighborVisionPoseSubscribers = new Vector<>();
        neighborLocalVelocitySubscribers = new Vector<>();
        neighborLocalVelocities = new Vector<>();
        neighborVisionPoses = new Vector<>();
        neighborNum = 0;
    }

    public boolean UpdataNeighborList(){

        neighborLocalVelocitySubscribers.clear();
        neighborLocalVelocities.clear();
        neighborVisionPoseSubscribers.clear();
        neighborVisionPoses.clear();

        int neighborIndex = 0;
        for(int droneIndex = 1;;droneIndex++) {
            try {
                if (!thisDroneName.equals("drone" + droneIndex)
                && !masterStateClient.lookupNode("/drone" + droneIndex + "/state_machine").isOpaque()) {

                    log.info("new neighbor found,drone name: drone" + droneIndex);

                    Subscriber<PoseStamped> newPoseSubscriber = node.newSubscriber("/drone" + droneIndex + "/mavros/vision_pose/pose", PoseStamped._TYPE);
                    neighborVisionPoseSubscribers.add(newPoseSubscriber);
                    Subscriber<TwistStamped> newVelocitySubscriber = node.newSubscriber("/drone" + droneIndex + "/mavros/local_position/velocity", TwistStamped._TYPE);
                    neighborLocalVelocitySubscribers.add(newVelocitySubscriber);

                    neighborLocalVelocities.add(new double[3]);
                    neighborVisionPoses.add(new double[3]);
                    final int temp_index = neighborIndex;
                    MessageListener<TwistStamped> localVelocityListener = new MessageListener<TwistStamped>() {
                        @Override
                        public void onNewMessage(TwistStamped twistStamped) {
                            neighborLocalVelocities.elementAt(temp_index)[0] = twistStamped.getTwist().getLinear().getX();
                            neighborLocalVelocities.elementAt(temp_index)[1] = twistStamped.getTwist().getLinear().getY();
                            neighborLocalVelocities.elementAt(temp_index)[2] = twistStamped.getTwist().getLinear().getZ();

                        }
                    };
                    neighborLocalVelocitySubscribers.elementAt(neighborIndex).addMessageListener(localVelocityListener);

                    MessageListener<PoseStamped> visionPoseListener = new MessageListener<PoseStamped>() {
                        @Override
                        public void onNewMessage(PoseStamped poseStamped) {
                            neighborVisionPoses.elementAt(temp_index)[0] = poseStamped.getPose().getPosition().getX();
                            neighborVisionPoses.elementAt(temp_index)[1] = poseStamped.getPose().getPosition().getY();
                            neighborVisionPoses.elementAt(temp_index)[2] = poseStamped.getPose().getPosition().getZ();
                        }
                    };
                    neighborVisionPoseSubscribers.elementAt(neighborIndex).addMessageListener(visionPoseListener);
                    neighborIndex++;
                }
            } catch (Exception e) {
                break;
            }
        }
        neighborNum = neighborIndex;
        return true;
    }

    public Vector<double[]> getNeighborVisionPoses(){return neighborVisionPoses;}
    public Vector<double[]> getNeighborLocalVelocities(){return neighborLocalVelocities;}
    public int getNeighborNum(){return neighborNum;}

}
