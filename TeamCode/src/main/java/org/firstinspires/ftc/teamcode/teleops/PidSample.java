package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor2;

@TeleOp
public class PidSample extends BaseRobot {
    Control.pid armController;
    double currentAngle;
    double output;
    public void init() {
        armController  = new Control.pid();
    }

    public void loop() {

        if(gamepad1.a) {

            currentAngle = Encoders.getMotorEnc(armLiftMotor1);
            output = armController.rotateWithPid(10, currentAngle);
            Control.motor.moveMotor(armLiftMotor1, output);
            Control.motor.moveMotor(armLiftMotor2, output);
        } else {
            Control.motor.moveMotor(armLiftMotor1, 0);
            Control.motor.moveMotor(armLiftMotor2, 0);
        }
    }
}
