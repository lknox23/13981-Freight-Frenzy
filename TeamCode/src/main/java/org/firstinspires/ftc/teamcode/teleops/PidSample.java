package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;

@TeleOp
public class PidSample extends BaseRobot {
    Control.pid armController;
    double currentAngle;
    double output;
    public void init() {
        super.init();
        armController  = new Control.pid();
        Encoders.resetMotorEnc(armLiftMotor1);
    }

    public void loop() {
        super.loop();
        if(gamepad1.b) {
            currentAngle = Control.auto.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
            output = armController.rotateWithPid(60, (currentAngle));
            Control.motor.moveMotor(armLiftMotor1, output);
        } else {
            Control.motor.moveMotor(armLiftMotor1, 0);
        }
        if(gamepad1.x) {
            Encoders.resetMotorEnc(armLiftMotor1);
        }

        telemetry.addData("arm 1 angle: ", Control.auto.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1)));

    }
}
