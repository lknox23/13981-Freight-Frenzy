package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_P;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor2;

@TeleOp
public class PidSample extends BaseRobot {
    Control.pid armController;
    double currentAngle;
    double output;

    double p=.03;
    double p1=0.001;
    double i=0;
    double d=0.05;
    public void init() {
        super.init();
        armController  = new Control.pid();
        Encoders.resetMotorEnc(armLiftMotor1);
        Encoders.resetMotorEnc(armLiftMotor2);
    }

    public void loop() {
        super.loop();

        currentAngle = Control.conversion.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));

        if(gamepad1.dpad_up) {
            //if (currentAngle<70)
                output = armController.rotateWithPid(60, (currentAngle), p, i, d);
            //else
            //    output = armController.rotateWithPid(60, currentAngle, p1, i, d);
        } else if (gamepad1.dpad_down) {
            output = armController.rotateWithPid(0, currentAngle, p1, i, d);
        }

        else if (gamepad1.triangle) {
            output = 0.1;
        } else if (gamepad1.cross) {
            output = -0.1;
        }

        else {
            output = 0;
        }

        Control.motor.moveMotor(armLiftMotor1, output);
        Control.motor.moveMotor(armLiftMotor2, output);

        if(gamepad1.circle) {
            Encoders.resetMotorEnc(armLiftMotor1);
            Encoders.resetMotorEnc(armLiftMotor2);
        }

        if (gamepad1.dpad_right) {
            p1+=.0001;
        } else if (gamepad1.dpad_left) {
            p1-=.0001;
        }

        telemetry.addData("p: ", p);
        telemetry.addData("p1: ", p1);
        telemetry.addData("i: ", i);
        telemetry.addData("d: ", d);
        telemetry.addData("arm 1 angle: ", Control.conversion.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1)));
        telemetry.addData("output: ", output);
    }
}
