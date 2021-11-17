package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.*;

import java.util.List;


/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class RoadRunnerAuto extends LinearOpMode {

    public void runOpMode() {
        Devices.initDevices(hardwareMap);
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(20, 30), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj1);

        // do something
/*
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(10)
                .build();
        drive.followTrajectory(traj2);

 */

        // do something else

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .back(10)
                .build();
        drive.followTrajectory(traj3);

        // done

        return;
    }
}
