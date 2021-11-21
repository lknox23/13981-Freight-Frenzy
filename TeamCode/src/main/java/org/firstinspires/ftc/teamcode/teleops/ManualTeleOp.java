package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_D;
import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_I;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor2;
import static org.firstinspires.ftc.teamcode.hardware.Devices.boxMover;
import static org.firstinspires.ftc.teamcode.hardware.Devices.intake;
import static org.firstinspires.ftc.teamcode.hardware.Devices.leftBackDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.leftFrontDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.slideLiftMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.spinner;

@TeleOp
public class ManualTeleOp extends BaseRobot {


    double currentAngle;
    Control.pid armController;
    boolean slowMode=false;
    boolean dumping = false;
    ElapsedTime dumpTimer;
    ElapsedTime slowModeCoolDown;
    ElapsedTime armLockCoolDown;
    ElapsedTime intakeTimer;
    int extensionPosition;
    double armPower;

    boolean holdArm=false;
    double holdPos;

    public static final double intakeRotationThreshold = 45; //degrees

    @Override
    public void init() {
        super.init();
        armController = new Control.pid();

        Devices.rightBackDriveMotor.setDirection(FORWARD);
        Devices.rightFrontDriveMotor.setDirection(FORWARD);
        leftBackDriveMotor.setDirection(REVERSE);
        leftFrontDriveMotor.setDirection(REVERSE);

        Devices.slideLiftMotor.setDirection(REVERSE);

        Encoders.resetMotorEnc(slideLiftMotor);
        Encoders.resetMotorEnc(armLiftMotor1);
        Encoders.resetMotorEnc(armLiftMotor2);


        dumpTimer = new ElapsedTime();
        slowModeCoolDown = new ElapsedTime();
        armLockCoolDown = new ElapsedTime();
        intakeTimer = new ElapsedTime();
        armPower=0;
        extensionPosition=0;

        boxMover.setPosition(0.32);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        //super.loop();

        // drive using tankanum

        if (gamepad1.left_stick_button && armLockCoolDown.seconds()>0.5) {
            holdArm= !holdArm;
            armLockCoolDown.reset();
        }
        if (gamepad2.touchpad && slowModeCoolDown.seconds()>0.5) {
            slowMode = !slowMode;
            slowModeCoolDown.reset();
        }
        if (slowMode) Control.drive.tankDrive(gamepad2.left_stick_y/5, gamepad2.right_stick_y/5);
        else Control.drive.tankDrive(gamepad2.left_stick_y, gamepad2.right_stick_y);


        currentAngle = Control.auto.armEncoderToAngle(armLiftMotor1.getCurrentPosition());


        //intake
        if (gamepad1.left_trigger>0.5) {
            intake.setPower(1);
        } else if (gamepad1.right_trigger>0.5) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        //control arm extension
        if (gamepad1.right_bumper) {
            slideLiftMotor.setPower(-0.2);
            extensionPosition = slideLiftMotor.getCurrentPosition();
        }
        else if (gamepad1.left_bumper && slideLiftMotor.getCurrentPosition()>0) {
            slideLiftMotor.setPower(0.2);
            extensionPosition = slideLiftMotor.getCurrentPosition();
        } else {
            slideLiftMotor.setPower(0);
        }
        if (currentAngle>70) {
            extensionPosition=200;
        }
        /*
        else {
            //Control.motor.linearSlideSetPosition(slideLiftMotor, extensionPosition);
            if (slideLiftMotor.getCurrentPosition()>extensionPosition+10) slideLiftMotor.setPower(0.2);
            else if (slideLiftMotor.getCurrentPosition()<extensionPosition-10) slideLiftMotor.setPower(-0.2);
            else slideLiftMotor.setPower(0);
        }

         */

        if (holdArm) {
            armPower = armController.rotateWithPid(holdPos, currentAngle, 0.1, K_I, K_D)/2;
        } else {
            armPower = -gamepad1.right_stick_y*0.5;
            holdPos=Control.auto.armEncoderToAngle(currentAngle);
        }
        Control.motor.moveMotor(Devices.armLiftMotor1, armPower);
        Control.motor.moveMotor(armLiftMotor2, armPower);


        //dumping
        if (gamepad1.square){ //start dumping
            dumping = true;
            dumpTimer.reset();
        }

        if (dumpTimer.seconds()>1 && dumpTimer.seconds()<2) { //end dumping
            stopDumping();
        }

        if (dumping) {
            intake.setPower(0.5); //1
            if (currentAngle<intakeRotationThreshold) {
                boxMover.setPosition(0);
            }
            else
                boxMover.setPosition(.32);
        } else {
            if (currentAngle<0) {
                boxMover.setPosition(0.32);
            }
            else if (currentAngle<intakeRotationThreshold) {
                boxMover.setPosition(.32);
            }
            else {
                boxMover.setPosition(0);
            }

        }

        //control spinner
        if (gamepad1.cross)
            Control.motor.moveMotor(spinner, 0.35); //negative if red
        else if (gamepad1.triangle)
            Control.motor.moveMotor(spinner, -0.35);
        else
            Control.motor.moveMotor(spinner, 0);

        //if (intakeTimer.seconds()>1)
        //    intake.setPower(0);

        telemetry.addData("arm extender encoder reading: ", slideLiftMotor.getCurrentPosition());
        telemetry.addData("current angle: ", currentAngle);
        telemetry.addData("extension position", extensionPosition);
        //telemetry.addData("arm mode: ", armMode);
        //telemetry.addData("box mover: ", boxMover.getPosition());
    }

    public void stopDumping() {
        dumping = false;
        //if (currentAngle>50)
            //intake.setPower(1);
        //armMode=1;
    }
}