package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
public class RedPark2 extends LinearOpMode{
    String parkingChoice;

    public void runOpMode() {
        Devices.initDevices(hardwareMap);
        //SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        telemetry.addData(">","Press dpad_down for storage parking");
        telemetry.addData(">","Press dpad_up for warehouse parking");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //parking choice
       parkingChoice = "warehouse";


        waitForStart();

        if (isStopRequested()) return;

        park();

    }

    public void park(){
        if (parkingChoice.equals("warehouse")) {
            //Control.auto.moveWithEncoder(0, 0.5);
            Control.auto.moveWithEncoder(10, 0.5);
            Control.auto.turnWithGyro(90, 0.5);
            Control.auto.moveWithEncoder(40, 0.5);
        }
        else if (parkingChoice.equals("storageUnit")){
            Control.auto.turnWithGyro(-75, 0.5);
            Control.auto.moveWithEncoder(22, 0.5);
        }
    }
}
