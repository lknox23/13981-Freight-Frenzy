package org.firstinspires.ftc.teamcode.autos;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.ConstantVariables;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

@Autonomous

public class myAuto2 extends LinearOpMode {

    public void runOpMode() {
        Devices.initDevices(hardwareMap);
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Control.auto.moveWithEncoder(10, 0.5);
        shippingHub();
        deliverDucks();


    }

    public void shippingHub() {
        Control.auto.moveWithEncoder(10, 0.5);
        Control.auto.turnWithGyro(45, -0.5);
        Control.auto.moveWithEncoder(10, 0.5);
        Control.auto.moveWithEncoder(10, -0.5);
        Control.auto.turnWithGyro(25, -0.5);
        Control.auto.moveWithEncoder(-23, 0.5);
        Control.auto.turnWithGyro(20, -0.5);
        Control.auto.moveWithEncoder(100, 0.5);
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


}