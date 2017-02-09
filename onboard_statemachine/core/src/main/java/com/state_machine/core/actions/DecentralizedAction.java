package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.droneState.DroneLanded;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.droneState.NeighborStateTracker;
import com.state_machine.core.providers.RosParamProvider;
import com.state_machine.core.providers.RosPublisherProvider;
import com.state_machine.core.providers.RosServiceProvider;
import geometry_msgs.TwistStamped;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.logging.Log;
import org.ejml.simple.SimpleMatrix;
import org.ros.exception.RemoteException;
import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;

import java.util.Vector;

/**
 * Created by Gao Changyu on 12/11/16.
 */
public class DecentralizedAction extends Action{

    private Log log;
    private DroneStateTracker stateTracker;
    private NeighborStateTracker neighborStateTracker;
    private Duration timeOut;
    private Publisher<TwistStamped> velocitySetpointPublisher;
    private TwistStamped velocityObjective;
    private ServiceClient<SetModeRequest, SetModeResponse> setModeService;
    private Vector<double[]> neighborVelocities;
    private Vector<double[]> neighborPoses;
    private double[] myVelocity;
    private double[] myPose;

    private RosParamProvider rosParamProvider;


    /*Algorithm related parameters*/
    private double D0;
    private double C;
    private double Lamda;
    private int Hp;
    private int Hu;
    private double Ts;
    private double Vmax;
    private Vector<double[]> xi_k;

    /*Algorithm related variables*/
    private SimpleMatrix xi;
    private SimpleMatrix I;
    private SimpleMatrix Z;
    private SimpleMatrix Ts_Matrix;
    private SimpleMatrix A;
    private SimpleMatrix B;
    private SimpleMatrix X;
    private SimpleMatrix PX;
    private SimpleMatrix PU;
    private SimpleMatrix E;
    private SimpleMatrix S;
    private SimpleMatrix e_ji;
    private SimpleMatrix R;
    private SimpleMatrix U;
    private SimpleMatrix W;
    private SimpleMatrix u;
    private SimpleMatrix des_velocity;
    private int Ni;

    /*Algorithm related Matrices*/

    public DecentralizedAction(Log log,
                               DroneStateTracker stateTracker,
                               NeighborStateTracker neighborStateTracker,
                               Duration timeOut,
                               RosParamProvider rosParamProvider,
                               RosPublisherProvider rosPublisherProvider,
                               RosServiceProvider rosServiceProvider){
        this.log = log;
        this.stateTracker = stateTracker;
        this.neighborStateTracker = neighborStateTracker;
        this.timeOut = timeOut;
        this.rosParamProvider = rosParamProvider;
        this.velocitySetpointPublisher = rosPublisherProvider.getSetpointVelocityPublisher();
        this.setModeService = rosServiceProvider.getSetFCUModeService();
        velocityObjective = velocitySetpointPublisher.newMessage();
        xi_k = new Vector<>();
        Ni = 0;

    }


    private void updateParams(){
        D0 = rosParamProvider.getD0();
        C = rosParamProvider.getC();
        Lamda = rosParamProvider.getLamda();
        Hp = rosParamProvider.getHp();
        Hu = rosParamProvider.getHu();
        Ts = rosParamProvider.getTs();
        Vmax = rosParamProvider.getVmax();
    }

    @Override
    public ActionStatus enterAction(Time time){
        status = ActionStatus.Inactive;
        if(stateTracker.getDroneLanded() == DroneLanded.InAir) {
            updateParams();
            neighborStateTracker.UpdataNeighborList();
            Ni = neighborStateTracker.getNeighborNum();
            //log.info("neighbor number:" + Ni);

            //construct the matrices
            I = SimpleMatrix.identity(Ni + 1);
            Z = new SimpleMatrix(Ni + 1, Ni + 1);
            Ts_Matrix = SimpleMatrix.identity(Ni + 1).scale(Ts);
            A = new SimpleMatrix(2 * Ni + 2, 2 * Ni + 2);
            B = new SimpleMatrix(2 * Ni + 2, Ni + 1);
            X = new SimpleMatrix(2 * (Ni + 1) * (Hp + 1), 2);
            PX = new SimpleMatrix(2 * (Ni + 1) * Hp, 2 * (Ni + 1));
            PU = new SimpleMatrix(2 * (Ni + 1) * Hp, (Ni + 1) * Hu);
            E = new SimpleMatrix((Ni + 1) * Hp, 2 * (Ni + 1) * Hp);
            S = new SimpleMatrix((Ni + 1) * Hp, 2);
            R = SimpleMatrix.identity(Hu * (Ni + 1)).scale(Lamda);
            //log.info("R "+R.toString());
            W = new SimpleMatrix(Ni + 1, (Ni + 1) * Hu);

            velocityObjective.getTwist().getLinear().setX(0);
            velocityObjective.getTwist().getLinear().setY(0);
            velocityObjective.getTwist().getLinear().setZ(0);
            velocityObjective.getHeader().setStamp(time);
            velocitySetpointPublisher.publish(velocityObjective);

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

    //pow >= 1
    private SimpleMatrix array_pow(SimpleMatrix mat, int pow){
        SimpleMatrix ret = mat.copy();
        for(int i = 1;i < pow;++i){
            ret = ret.mult(mat);
        }
        return ret;
    }

    @Override
    public ActionStatus loopAction(Time time) {
        if(time.subtract(timeStamp).compareTo(timeOut) > 0){
            return ActionStatus.Success;
        }else {

            /* get current state */
            neighborVelocities = neighborStateTracker.getNeighborLocalVelocities();
            neighborPoses = neighborStateTracker.getNeighborVisionPoses();
            myVelocity = stateTracker.getLocalVelocity();
            myPose = stateTracker.getVisionPosition();
            double[][] temp_array = new double[2*(Ni + 1)][3];
            xi_k.clear();
            xi_k.add(myPose);
            xi_k.addAll(neighborPoses);
            xi_k.add(myVelocity);
            xi_k.addAll(neighborVelocities);
            xi_k.toArray(temp_array);
            xi = new SimpleMatrix(temp_array);
            xi = xi.extractMatrix(0,SimpleMatrix.END,0,2);
            //log.info(xi.toString());
            /* generate algorithm related matrices*/


            Z.zero();

            A.insertIntoThis(0,0,I.combine(0,SimpleMatrix.END,Ts_Matrix));
            A.insertIntoThis(Ni + 1,0,Z.combine(0,SimpleMatrix.END,I));

            B.insertIntoThis(0,0,Z.combine(SimpleMatrix.END,0,Ts_Matrix));
            //log.info("Z: " + Z.toString());
            //log.info("Ts: " + Ts_Matrix.toString());
            //log.info("A: " + A.toString());
            //log.info("B: " + B.toString());

            X.insertIntoThis(0,0,xi);
            for(int k = 1; k < (Hp + 1);++k) {
                //log.info(X.extractMatrix((k - 1) * 2 * (Ni + 1), k * 2 * (Ni + 1), 0, SimpleMatrix.END).toString());
                //log.info(A.toString());
                X.insertIntoThis(k * 2 * (Ni + 1), 0, A.mult(X.extractMatrix((k - 1) * 2 * (Ni + 1), k * 2 * (Ni + 1), 0, SimpleMatrix.END)));
            }

            //log.info("X: " + X.toString());
            for(int k = 1; k<(Hp + 1);++k){
                PX.insertIntoThis((k-1)*2*(Ni+1),0,array_pow(A,k));
            }


            for(int k = 1; k < Hu; ++k){
                for(int l = k;l<Hp+1;++l){
                    PU.insertIntoThis((l-1)*2*(Ni+1),(k-1)*(Ni +1),array_pow(A,l-k).mult(B));
                }
            }

            for(int l = Hu; l<(Hp+1);++l){
                for(int k = 0; k <(l-Hu+1);++k ){
                    PU.insertIntoThis((l-1)*2*(Ni +1),(Hu-1)*(Ni+1),array_pow(A,l-k).mult(B).plus(PU.extractMatrix((l-1)*2*(Ni+1),l*2*(Ni+1),(Hu-1)*(Ni+1),Hu*(Ni+1))));
                }
            }



            SimpleMatrix temp_matrix;
            for(int k =1;k < (Hp +1); ++k){
                E.set(k*(Ni+1)-1,(2*k - 1)*(Ni+1),C*(1/(Ni+1) - 1));
                temp_matrix = new SimpleMatrix(1,Ni);
                temp_matrix.set(C*(1/(Ni+1)));
                E.insertIntoThis(k*(Ni+1)-1,(2*k-1)*(Ni+1)+1,temp_matrix);

                temp_matrix = new SimpleMatrix(Ni,1);
                temp_matrix.set(-1);
                E.insertIntoThis((k-1)*(Ni+1),(k-1)*2*(Ni+1),temp_matrix);
                E.insertIntoThis((k-1)*(Ni+1), (k-1)*2*(Ni+1)+1,SimpleMatrix.identity(Ni));

                for(int l = 2;l<((Ni+1)+1);++l){
                     e_ji = X.extractVector(true,k*2*(Ni+1)+l-1).plus(X.extractVector(true,k*2*(Ni+1)).negative());
                    S.insertIntoThis((k-1)*(Ni+1)+l-2,0, e_ji.scale(D0/e_ji.normF()));
                }
            }

            //log.info("PX: "+PX.toString());
            //log.info("PU: "+PU.toString());
            //log.info("E: "+E.toString());
            //log.info("S: "+S.toString());
            /*Do the MPC and update states*/


            U = E.mult(PU).transpose().mult(E).mult(PU).plus(R).solve(E.mult(PU).transpose().mult(E.mult(PX).mult(xi).plus(S.negative()))).negative();


            W.insertIntoThis(0,0,SimpleMatrix.identity(Ni+1));
            u = W.mult(U);
            //log.info(u.toString());

            temp_matrix = new SimpleMatrix(1,2);
            temp_matrix.setRow(0,0,myVelocity[0],myVelocity[1]);
            des_velocity = u.extractVector(true,0).scale(Ts);
            des_velocity.plus(temp_matrix);
            double des_speed = des_velocity.normF();

            if(des_speed>Vmax){
                des_velocity = des_velocity.scale(Vmax/des_speed);
            }
            //log.info("des_speed: " + des_speed);
            //log.info(des_velocity.toString());

            //send to drone
            velocityObjective.getTwist().getLinear().setX(des_velocity.get(0,0));
            velocityObjective.getTwist().getLinear().setY(des_velocity.get(0,1));
            velocityObjective.getTwist().getLinear().setZ(0);
            velocityObjective.getHeader().setStamp(time);
            velocitySetpointPublisher.publish(velocityObjective);

            /*
            for (int i = 0; i < neighborPoses.size(); ++i) {
                log.info("neighbor" + (i + 1) + " pose:" + neighborPoses.elementAt(i)[0] + ", " + neighborPoses.elementAt(i)[1] + ", " + neighborPoses.elementAt(i)[2]);
            }
            for (int i = 0; i < neighborVelocities.size(); ++i) {
                log.info("neighbor" + (i + 1) + " vel:" + neighborVelocities.elementAt(i)[0] + ", " + neighborVelocities.elementAt(i)[1] + ", " + neighborVelocities.elementAt(i)[2]);
            }
            */
            return ActionStatus.Waiting;
        }
    }
}
