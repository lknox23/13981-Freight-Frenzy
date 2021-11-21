package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class FeedforwardController {
    double F;
    double k_V;
    double k_A;

    double maximumSpeed;
    double maxAcceleration;

    private double previousError;
    private double previousTime;
    private double previousVelocity;

    private double currentError;
    private double currentVelocity;
    private double currentAcceleration;
    private double currentTime;

    private double goal;

    private double goalVelocity;
    private double goalAcceleration;


    private boolean isPidf;
    private Control.pid pidController;

    ElapsedTime timer;

    public FeedforwardController(//double f,
                                 double v, double a, double maxSpeed, double maxAccel, boolean pidf) {
        //F=f;
        F=0;
        k_V=v;
        k_A=a;
        maximumSpeed=maxSpeed;
        maxAcceleration=maxAccel;

        previousError=0;
        previousTime=0;
        previousVelocity=0;
        timer = new ElapsedTime();

        isPidf=pidf;
        if(isPidf) {
            Control.pid pidController = new Control.pid();
        }
    }
    public double getOutput (double currentPosition, double goalPosition) {
        goal = goalPosition;

        currentTime = timer.seconds();
        currentError = goal-currentPosition;
        currentVelocity=(currentError-previousError)/(currentTime-previousTime);
        currentAcceleration = (currentVelocity-previousVelocity)/(currentTime-previousTime);

        double output;

        //start
        
        int direction_multiplier = 1;

        if (currentError < 0) {
            direction_multiplier = -1;
        }
        if (Math.abs(currentVelocity) < maximumSpeed){
            goalVelocity = currentVelocity + direction_multiplier * maxAcceleration * (currentTime - previousTime);
            goalAcceleration = maxAcceleration;
        } else {
            goalVelocity = maximumSpeed;
            goalAcceleration = 0;
        }

        if (currentError <= (goalVelocity * goalVelocity) / (2 * maxAcceleration)) {
            goalVelocity = currentVelocity - direction_multiplier * maxAcceleration * (currentTime - previousTime);
            goalAcceleration = -maxAcceleration;
        }
                //end

        double velError = goalVelocity-currentVelocity;
        double accError = goalAcceleration-currentAcceleration;

        if (isPidf) {
            F=pidController.rotateWithPid(goalVelocity, currentVelocity, 0, 0, 0);
        }

        output = F + k_V * velError + k_A * accError;


        previousError=currentError;
        previousTime=currentTime;
        previousVelocity=currentVelocity;

        return output;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("error: ", currentError);
        telemetry.addData("velocity: ", currentVelocity);
        telemetry.addData("acceleration: ", currentAcceleration);

        telemetry.addData("goal velocity: ", goalVelocity);
        telemetry.addData("goal acceleration: ", goalAcceleration);
    }
}
