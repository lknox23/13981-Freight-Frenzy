package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.ConstantVariables;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor2;
import static org.firstinspires.ftc.teamcode.hardware.Devices.intakeServo;
import static org.firstinspires.ftc.teamcode.hardware.Devices.slideLiftMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.spinner;

@TeleOp
//testing 123
public class TestingTeleOp extends BaseRobot {
    double currentAngle;
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        // drive using tankanum
        Control.drive.tankanumDrive(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_x);

        // control arm rotation
        if (gamepad1.dpad_up) {
            if (armLiftMotor1.getCurrentPosition()<30) {
                Control.motor.moveMotor(Devices.armLiftMotor1, 0.1);
                Control.motor.moveMotor(Devices.armLiftMotor2, 0.1);
            } else {
                Control.motor.moveMotor(Devices.armLiftMotor1, 1);
                Control.motor.moveMotor(Devices.armLiftMotor2, 1);
            }
        } else if (gamepad1.dpad_down) {
            if (armLiftMotor1.getCurrentPosition()>30) {
                Control.motor.moveMotor(Devices.armLiftMotor1, -0.5);
                Control.motor.moveMotor(Devices.armLiftMotor2, -0.5);
            } else {
                Control.motor.moveMotor(Devices.armLiftMotor1, -1);
                Control.motor.moveMotor(Devices.armLiftMotor2, -1);
            }
        } else {
            Control.motor.moveMotor(Devices.armLiftMotor1, 0);
            Control.motor.moveMotor(Devices.armLiftMotor2, 0);
        }

        //control arm extension
        if (gamepad1.left_bumper)
            Control.motor.moveMotor(slideLiftMotor, 1);
        else if (gamepad1.right_bumper)
            Control.motor.moveMotor(slideLiftMotor, -1);
        else
            Control.motor.moveMotor(slideLiftMotor, 0);

        //control spinner
        if (gamepad1.a)
            Control.motor.moveMotor(spinner, 1);
        else if (gamepad1.b)
            Control.motor.moveMotor(spinner, -1);
        else
            Control.motor.moveMotor(spinner, 0);

        //intake
        if (gamepad1.dpad_left) {
            intakeServo.setPosition(0);
        } else if (gamepad1.dpad_right) {
            intakeServo.setPosition(1);
        }


        telemetry.addData("arm econder reading 1: ", armLiftMotor1.getCurrentPosition());
        telemetry.addData("arm econder reading 2: ", armLiftMotor2.getCurrentPosition());
        currentAngle = armLiftMotor1.getCurrentPosition()/ ConstantVariables.ARM_ROTATE_PPR* 360;
        telemetry.addData("current angle: ", currentAngle);

    }
}

