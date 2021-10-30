package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;


public class myAuto extends LinearOpMode {

    /*
     * Steps:
     * 1. Deliver preloaded box to alliance shipping hub
     * 2. Ducks
     * 3. Park
     * */

    int duckPositionIndex;
    SampleTankDrive drive;
    double coeff=1;
    @Override
    public void runOpMode() {
        initializeStuff();

        waitForStart();
    }

    public void initializeStuff() {
        Devices.initDevices(hardwareMap);
        Control.auto.initTF("FreightFrenzy_DM.tflite", new String[]{
                "Duck",
                "Marker"
        }, 1.0, hardwareMap);
        Control.sensor.initGyro();
        drive = new SampleTankDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(10*coeff, 10*coeff, Math.toRadians(90)));
    }

    public void detectDucks() {
        Recognition duck = Control.auto.getDuck(telemetry);

        double duckPositionAngle = duck.estimateAngleToObject(AngleUnit.DEGREES);
        duckPositionIndex = Control.auto.getDuckPositionIndexThree(duckPositionAngle);
    }

    public void goToShippingHub(){
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-13, -40), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj1);
    }

    public void placePreloaded() {

    }

    public void goToCarousel() {
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-61, -55), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj2);

    }

    public void deliverDucks() {
        ElapsedTime runtime;
        runtime = new ElapsedTime();
        double oldTime;
        oldTime = 0;
        double dT = runtime.milliseconds() - oldTime;

        while (dT <= 2000) {
            dT = runtime.milliseconds() - oldTime;
            Control.motor.moveMotor(Devices.spinner, 0.5);
        }
    }

    public void park(){
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(38, -61), Math.toRadians(0))
                .build();
        drive.followTrajectory(traj3);


    }
}
