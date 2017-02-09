package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.actions.util.Waypoint;
import com.state_machine.core.droneState.DroneLanded;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.providers.RosPublisherProvider;
import com.state_machine.core.providers.RosServiceProvider;
import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.lang.ObjectUtils;
import org.ros.exception.RemoteException;
import org.ros.internal.message.Message;
import org.ros.internal.message.RawMessage;
import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import std_msgs.Header;
import org.apache.commons.logging.Log;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

/**
 * Created by Gao Changyu on 10/23/16.
 */
public class PX4FlyToAction extends Action {

    private PoseStamped objective;
    private DroneStateTracker stateTracker;
    private Publisher<PoseStamped> setpointPositionLocalPub;
    private Duration timeOut;
    private ServiceClient<SetModeRequest, SetModeResponse> setModeService;
    private int seq;
    private Log logger;

    public PX4FlyToAction(Log logger,
                          Waypoint objective,
                          DroneStateTracker stateTracker,
                          RosPublisherProvider rosPublisherProvider,
                          Duration timeOut,
                          RosServiceProvider serviceProvider,
                          int seq){
        this.logger =logger;
        this.seq = seq;
        setModeService = serviceProvider.getSetFCUModeService();
        this.setpointPositionLocalPub = rosPublisherProvider.getSetpointPositionLocalPublisher();

        this.objective = this.setpointPositionLocalPub.newMessage();

        //temporarily store the objective in reference with world frame
        this.objective.getPose().getPosition().setX(objective.getX());
        this.objective.getPose().getPosition().setY(objective.getY());
        this.objective.getPose().getPosition().setZ(objective.getZ());

        this.objective.getHeader().setSeq(seq);

        this.stateTracker = stateTracker;

        this.timeOut = timeOut;
    }

    private double calculateDistance(Pose objective, double[] localPosition){
        return sqrt(pow((objective.getPosition().getX() - localPosition[0]) ,2) + pow((objective.getPosition().getY() - localPosition[1]),2) + pow((objective.getPosition().getZ() - localPosition[2]),2));
    }

    @Override
    public ActionStatus enterAction(Time time){
        status = ActionStatus.Inactive;

        if(stateTracker.getDroneLanded() == DroneLanded.InAir){
            //adjust the object to local
            double temp = objective.getPose().getPosition().getX() - stateTracker.getLocalOrigin()[0];
            objective.getPose().getPosition().setX(-(objective.getPose().getPosition().getY()-stateTracker.getLocalOrigin()[1]));
            objective.getPose().getPosition().setY(temp);

            //objective.getPose().getPosition().setX(objective.getPose().getPosition().getY() - stateTracker.getLocalOrigin()[1]);
            //objective.getPose().getPosition().setY(objective.getPose().getPosition().getX() - stateTracker.getLocalOrigin()[0]);
            objective.getPose().getPosition().setZ(objective.getPose().getPosition().getZ() - stateTracker.getLocalOrigin()[2]);
            //logger.warn("actual local origin at "
            //        + stateTracker.getLocalOrigin()[0] + ","
            //        + stateTracker.getLocalOrigin()[1] + ","
            //        + stateTracker.getLocalOrigin()[2] + ".");

            objective.getHeader().setStamp(time);
            setpointPositionLocalPub.publish(objective);
            seq++;

            if (!stateTracker.getFCUMode().equals("OFFBOARD")){
                if(!setModeService.isConnected()) return ActionStatus.ConnectionFailure;

                SetModeRequest request = setModeService.newMessage();
                request.setCustomMode("OFFBOARD");

                ServiceResponseListener<SetModeResponse> listener = new ServiceResponseListener<SetModeResponse>() {
                    @Override
                    public void onSuccess(SetModeResponse setModeResponse) {
                        status = ActionStatus.Success;
                    }

                    @Override
                    public void onFailure(RemoteException e) {
                        status = ActionStatus.Failure;
                    }
                };
                setModeService.call(request, listener);
            }

            timeStamp = time;

            status = ActionStatus.Inactive;
            return ActionStatus.Success;
        }else{
            return ActionStatus.Failure;
        }
    }

    @Override
    public ActionStatus loopAction(Time time) {
        if(calculateDistance(objective.getPose(),stateTracker.getLocalPosition()) < 0.5
                && stateTracker.getDroneLanded() == DroneLanded.InAir){
            status = ActionStatus.Success;
            return status;
        }else if(time.subtract(timeStamp).compareTo(timeOut) >= 0){
            status = ActionStatus.Failure;
            return status;
        }else{
            objective.getHeader().setStamp(time);
            objective.getHeader().setSeq(seq);
            setpointPositionLocalPub.publish(objective);
            seq++;
            status = ActionStatus.Waiting;
            return status;
        }
    }
}
