package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.droneState.NeighborStateTracker;
import org.apache.commons.logging.Log;
import org.ros.message.Duration;
import org.ros.message.Time;

import java.util.Vector;

/**
 * Created by Gao Changyu on 12/11/16.
 */
public class DecentralizedAction extends Action{

    private Log log;
    private DroneStateTracker stateTracker;
    private NeighborStateTracker neighborStateTracker;
    private Duration timeOut;
    private Vector<double[]> neighborVelocities;
    private Vector<double[]> neighborPoses;

    public DecentralizedAction(Log log,
                        DroneStateTracker stateTracker,
                        NeighborStateTracker neighborStateTracker,
                        Duration timeOut){
        this.log = log;
        this.stateTracker = stateTracker;
        this.neighborStateTracker = neighborStateTracker;
        this.timeOut = timeOut;
    }

    @Override
    public ActionStatus enterAction(Time time){
        neighborStateTracker.UpdataNeighborList();
        timeStamp = time;
        return ActionStatus.Success;
    }

    @Override
    public ActionStatus loopAction(Time time) {
        if(time.subtract(timeStamp).compareTo(timeOut) > 0){
            return ActionStatus.Success;
        }else {
            neighborVelocities = neighborStateTracker.getNeighborLocalVelocities();
            neighborPoses = neighborStateTracker.getNeighborVisionPoses();
            for (int i = 0; i < neighborPoses.size(); ++i) {
                log.info("neighbor" + (i + 1) + " pose:" + neighborPoses.elementAt(i)[0] + ", " + neighborPoses.elementAt(i)[1] + ", " + neighborPoses.elementAt(i)[2]);
            }
            for (int i = 0; i < neighborVelocities.size(); ++i) {
                log.info("neighbor" + (i + 1) + " vel:" + neighborVelocities.elementAt(i)[0] + ", " + neighborVelocities.elementAt(i)[1] + ", " + neighborVelocities.elementAt(i)[2]);
            }
            return ActionStatus.Waiting;
        }
    }
}
