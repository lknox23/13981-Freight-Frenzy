package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.ConstantVariables;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.boxMover;
import static org.firstinspires.ftc.teamcode.hardware.Devices.intake;
import static org.firstinspires.ftc.teamcode.hardware.Devices.slideLiftMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.spinner;

@TeleOp
//testing 123
public class TestingTeleOp extends BaseRobot {
    double currentAngle;
    double restPosition;
    Control.pid armController;

    @Override
    public void init() {
        super.init();
        armController = new Control.pid();
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
        currentAngle = Control.auto.armEncoderToAngle(armLiftMotor1.getCurrentPosition());
        // control arm rotation
        if (gamepad1.dpad_up && currentAngle<135) {
                double output = armController.rotateWithPid(90, (currentAngle));
                Control.motor.moveMotor(Devices.armLiftMotor1, 1);
                restPosition = currentAngle;
        } else if (gamepad1.dpad_down &&currentAngle>-30) {
                double output = armController.rotateWithPid(0, currentAngle);
                Control.motor.moveMotor(Devices.armLiftMotor1, -1);
                restPosition = currentAngle;
        } else {
                double output = armController.rotateWithPid(restPosition, currentAngle);
            Control.motor.moveMotor(Devices.armLiftMotor1, output);
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
            Control.motor.moveMotor(spinner, -0.3);
        else
            Control.motor.moveMotor(spinner, 0);

        //intake
        if (gamepad1.dpad_left) {
            intake.setPower(-1);
        } else if (gamepad1.dpad_right) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0);
        }

        if(gamepad1.x) {
            boxMover.setPosition(0);
        } else if (gamepad1.b) {
            boxMover.setPosition(1.0);
        }

        telemetry.addData("arm econder reading 1: ", armLiftMotor1.getCurrentPosition());
        telemetry.addData("arm extender encoder reading: ", slideLiftMotor.getCurrentPosition());
        currentAngle = armLiftMotor1.getCurrentPosition()/ ConstantVariables.K_ARM_ROTATE_PPR * 360 * ConstantVariables.K_ARM_GEAR_RATIO;
        telemetry.addData("current angle: ", currentAngle);
    }
}

