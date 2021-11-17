package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.util.ElapsedTime;

public class FeedforwardController {
    double F = 0;
    double k_V=0;
    double k_A=0;

    double previousError;
    double previousTime;
    double previousVelocity;
    double previousAcceleration;

    double goal;

    ElapsedTime currentTime;

    public FeedforwardController(double goalPosition) {
        previousError=0;
        previousTime=0;
        previousVelocity=0;
        previousAcceleration=0;
        currentTime = new ElapsedTime();
        goal = goalPosition;
    }

    public double[] updateValues(double currentPosition) {
        double time = currentTime.seconds();
        double currentError = goal-currentPosition;
        double currentVelocity=(currentError-previousError)/(time-previousTime);
        double currentAcceleration = (currentVelocity-previousVelocity)/(time-previousTime);

        previousError=currentError;
        previousTime=time;
        previousVelocity=currentVelocity;
        previousAcceleration=currentAcceleration;

        return new double[]{currentVelocity, currentAcceleration};
    }

    public double getOutput (double currentPosition, MotionModel motionProfile) {
        double[] temp = updateValues(currentPosition);
        double velocity = temp[0];
        double acceleration = temp[1];

        //double velError = motionProfile.getV(velocity)-velocity;
        //double accError = motionProfile.getA(acceleration)-acceleration;

        //return F + k_V * velError + k_A * accError;
        return 0;
    }

    public static class MotionModel {
        double maxV;
        double maxA;
        public MotionModel(double maxVelocity, double maxAcceleration) {
            maxV=maxVelocity;
            maxA=maxAcceleration;

        }
    }
}
