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

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

/**
 * Created by firefly on 10/23/16.
 */
public class PX4FlyToAction extends Action {

    private PoseStamped objective;
    private DroneStateTracker stateTracker;
    private Publisher<PoseStamped> setpointPositionLocalPub;
    private Duration timeOut;
    private ServiceClient<SetModeRequest, SetModeResponse> setModeService;
    private int seq;

    public PX4FlyToAction(Waypoint objective,
                          DroneStateTracker stateTracker,
                          RosPublisherProvider rosPublisherProvider,
                          Duration timeOut,
                          RosServiceProvider serviceProvider,
                          int seq){
        this.seq = seq;
        setModeService = serviceProvider.getSetFCUModeService();
        this.setpointPositionLocalPub = rosPublisherProvider.getSetpointPositionLocalPublisher();

        this.objective = this.setpointPositionLocalPub.newMessage();

        this.objective.getPose().getPosition().setX(objective.getX());
        this.objective.getPose().getPosition().setY(objective.getY());
        this.objective.getPose().getPosition().setZ(objective.getZ());
        this.objective.getHeader().setSeq(seq);

        this.stateTracker = stateTracker;

        this.timeOut = timeOut;
    }

    private double calculateDistance(Pose a, Pose b){
        return sqrt(pow((a.getPosition().getX() - b.getPosition().getX()),2) + pow((a.getPosition().getY() - b.getPosition().getY()),2) + pow((a.getPosition().getZ() - b.getPosition().getZ()),2));
    }

    @Override
    public ActionStatus enterAction(Time time){
        status = ActionStatus.Inactive;

        if(stateTracker.getDroneLanded() == DroneLanded.InAir){
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
        if(calculateDistance(stateTracker.getLocalPosition(),objective.getPose()) < 0.5
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
