package org.firstinspires.ftc.teamcode.autos;


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

public class myAutoRed2 extends LinearOpMode {

    SamplePipeline pipeline;
    int duckPositionIndex;
    String parkingChoice;

    public void runOpMode() {
        Devices.initDevices(hardwareMap);
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        telemetry.addData(">","Press dpad_down for storage parking");
        telemetry.addData(">","Press dpad_up for warehouse parking");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //parking choice
        if (gamepad1.dpad_down){
            parkingChoice = "storageUnit";
        }
        else if(gamepad1.dpad_up){
            parkingChoice = "warehouse";
        }


        waitForStart();

        if (isStopRequested()) return;

        Control.auto.moveWithEncoder(10, 0.5);
        detectDucks();
        shippingHub();
        placePreloaded();
        carousel();
        deliverDucks();
        park();


    }

    public void shippingHub() {
        Control.auto.moveWithEncoder(10, 0.5);
        Control.auto.turnWithGyro(-45, -0.5);
        Control.auto.moveWithEncoder(10, 0.5);
    }
    public void carousel() {
        Control.auto.moveWithEncoder(10, -0.5);
        Control.auto.turnWithGyro(-25, -0.5);
        Control.auto.moveWithEncoder(-23, 0.5);
    }
    public void park(){
        if (parkingChoice.equals("warehouse")) {
            Control.auto.moveWithEncoder(35, 0.5);
            Control.auto.turnWithGyro(-20, -0.5);
            Control.auto.moveWithEncoder(80, 0.5);
        }
        else if (parkingChoice.equals("storageUnit")){
            Control.auto.turnWithGyro(-75, 0.5);
            Control.auto.moveWithEncoder(22, 0.5);
        }
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

    public void detectDucks() {

        /*
        Recognition duck = Control.auto.getDuck(telemetry);

        double duckPositionAngle = duck.estimateAngleToObject(AngleUnit.DEGREES);
        duckPositionIndex = Control.auto.getDuckPositionIndexThree(duckPositionAngle);
         */


        if (pipeline.inRegion1())
            duckPositionIndex = 0;
        else if (pipeline.inRegion2())
            duckPositionIndex = 1;
        else
            duckPositionIndex = 3;
    }

    public void placePreloaded() {
        double currentAngle = Control.auto.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
        double output;
        int angle;
        if (duckPositionIndex==0) angle = 0;
        else if (duckPositionIndex==1) angle = 30;
        else angle = 60;

        Control.pid armController = new Control.pid();

        boxMover.setPosition(1);

        while (opModeIsActive() && currentAngle<angle-5){
            currentAngle = Control.auto.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
            output = armController.rotateWithPid(angle, (currentAngle), K_P, K_I, K_D);
            Control.motor.moveMotor(armLiftMotor1, output);
            if (slideLiftMotor.getCurrentPosition()<100) slideLiftMotor.setPower(0.2);
        }

        ElapsedTime timer = new ElapsedTime();
        telemetry.addLine("point reached");
        boxMover.setPosition(0);

        while (opModeIsActive()&&timer.seconds() < 1 ){
            currentAngle = Control.auto.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
            output = armController.rotateWithPid(angle, (currentAngle), K_P, K_I, K_D);
            Control.motor.moveMotor(armLiftMotor1, output);
            if (slideLiftMotor.getCurrentPosition()<100) slideLiftMotor.setPower(0.2);
        }
    }

}