package org.firstinspires.ftc.teamcode.autos;

import
        static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;


import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_D;
import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_I;
import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_P;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.boxMover;
import static org.firstinspires.ftc.teamcode.hardware.Devices.slideLiftMotor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.ConstantVariables;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;
import org.firstinspires.ftc.teamcode.hardware.SamplePipeline;

@Autonomous
public class BluePark2 extends LinearOpMode{
    String parkingChoice;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    public void runOpMode() {
        Devices.initDevices(hardwareMap);


        /*telemetry.addData(">","Press dpad_down for storage parking");
        telemetry.addData(">","Press dpad_up for warehouse parking");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();*/

        //parking choice
        //if (gamepad1.dpad_down){
        //    parkingChoice = "storageUnit";
        //}
        //else if(gamepad1.dpad_up){
            parkingChoice = "warehouse";
        //}

        waitForStart();

        if (isStopRequested()) return;

        park();

    }

    public void park(){
        Pose2d startingPose = new Pose2d(0, 0, 0);
        if (parkingChoice.equals("warehouse")) {
            Trajectory traj1 = drive.trajectoryBuilder(startingPose)
                    .splineTo(new Vector2d(20, 30), Math.toRadians(90))
                    .build();
            drive.followTrajectory(traj1);
        }
        else if (parkingChoice.equals("storageUnit")){
            Control.auto.turnWithGyro(75, 0.5);
            Control.auto.moveWithEncoder(22, 0.5);
        }
    }
}
