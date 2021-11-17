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
    public double getOutput (double currentPosition, MotionModel motionProfile) {
        double time = currentTime.seconds();
        double currentError = goal-currentPosition;
        double currentVelocity=(currentError-previousError)/(time-previousTime);
        double currentAcceleration = (currentVelocity-previousVelocity)/(time-previousTime);
        double maximumSpeed;
        double maxAcceleration;
        double goalVelocity;
        double goalAcceleration;
        //start
        
        int direction_multiplier = 1;

        if (currentError < 0) {
            direction_multiplier = -1;
        }
        if (Math.abs(currentVelocity) < maximumSpeed){
            K_v = currentVelocity + direction_multiplier * maxAcceleration * (time - previousTime);
            K_a = maxAcceleration;
        }
   #if maximum speed has been reached, stay there for now
   else:
        outputVelocity = MAXIMUM_SPEED
        outputAcceleration = 0

   #if we are close enough to the object to begin slowing down
        if position_error <= (output_velocity * output_velocity) / (2 * MAX_ACCELERATION)):
        output_velocity = current_velocity - direction_multiplier * MAX_ACCELERATION * (current_time - previous_time)
        output_acceleration = -MAX_ACCELERATION

        previous_time = current_time
                //end

        double velError = motionProfile.getV(velocity)-velocity;
        double accError = motionProfile.getA(acceleration)-acceleration;

        output = F + k_V * velError + k_A * accError;


        previousError=currentError;
        previousTime=time;
        previousVelocity=currentVelocity;
        previousAcceleration=currentAcceleration;

        return output;
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
