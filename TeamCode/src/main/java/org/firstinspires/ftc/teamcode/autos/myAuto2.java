package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.Devices;

@Autonomous

public class myAuto2 extends LinearOpMode {

    public void runOpMode() {
        Devices.initDevices(hardwareMap);
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;




    }
