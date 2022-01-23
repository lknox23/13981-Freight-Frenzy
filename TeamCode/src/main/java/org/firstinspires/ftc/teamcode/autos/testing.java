package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_D;
import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_I;
import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_P;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor2;
import static org.firstinspires.ftc.teamcode.hardware.Devices.boxMover;
import static org.firstinspires.ftc.teamcode.hardware.Devices.intakeMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.slideLiftMotor;


@Config
@Autonomous
public class testing extends LinearOpMode {
    public static int duckPositionIndex=0;
    public static int angle = 40;
    public static int extension = 250;
    public static double restPower = 0.15;
    //30, 250, .15
    //0, 0, 100, .1
    //45, 400, .2


    public void runOpMode() {
        Devices.initDevices(hardwareMap);
        waitForStart();
        placePreloaded(angle, extension, restPower);
        //support.setPosition(0);
        /*
        boxMover.setPosition(0);
        boxMover.setPosition(1);
        boxMover.setPosition(0.5);
         */

    }

    public void placePreloaded(double angle, int extension, double restPower) {
        double currentAngle = Control.conversion.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
        double output;
        //int angle;
        /*
        if (duckPositionIndex==0) angle = 0;
        else if (duckPositionIndex==1) angle = 30;
        else angle = 60;


         */
        Control.pid armController = new Control.pid();

        boxMover.setPosition(1);

        while (opModeIsActive() && currentAngle<angle-5){
            currentAngle = Control.conversion.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
            output = armController.rotateWithPid(angle, (currentAngle), K_P, K_I, K_D);
            Control.motor.moveMotor(armLiftMotor2, output/2);
            Control.motor.moveMotor(armLiftMotor1, output/2);
            if (slideLiftMotor.getCurrentPosition()<250) slideLiftMotor.setPower(0.3);
            else slideLiftMotor.setPower(restPower);
        }

        ElapsedTime timer = new ElapsedTime();
        telemetry.addLine("point reached");
        telemetry.update();
        boxMover.setPosition(0);

        while (opModeIsActive()&&timer.seconds() < 3){
            currentAngle = Control.conversion.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
            output = armController.rotateWithPid(angle, (currentAngle), K_P, K_I, K_D);
            Control.motor.moveMotor(armLiftMotor2, output/2);
            Control.motor.moveMotor(armLiftMotor1, output/2);
            if (slideLiftMotor.getCurrentPosition()<extension) slideLiftMotor.setPower(0.4);
            else slideLiftMotor.setPower(restPower);
        }

        while (opModeIsActive()&&timer.seconds() < 5){
            currentAngle = Control.conversion.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
            output = armController.rotateWithPid(angle, (currentAngle), K_P, K_I, K_D);
            Control.motor.moveMotor(armLiftMotor2, output/2);
            Control.motor.moveMotor(armLiftMotor1, output/2);
            //if (slideLiftMotor.getCurrentPosition()<300) slideLiftMotor.setPower(-0.25);
            //else
                slideLiftMotor.setPower(-0.2);
        }
    }
}
