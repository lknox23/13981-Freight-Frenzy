package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.ConstantVariables;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_D;
import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_I;
import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_P;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.boxMover;
import static org.firstinspires.ftc.teamcode.hardware.Devices.intakeMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.leftBackDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.leftFrontDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.slideLiftMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.spinner;

@TeleOp
public class TestingTeleOp extends BaseRobot {


    double currentAngle;
    double restPosition=0;
    Control.pid armController;
    boolean slowMode=false;
    boolean dumping = false;
    ElapsedTime dumpTimer;
    ElapsedTime slowModeCoolDown;
    ElapsedTime armModeCoolDown;
    ElapsedTime intakeTimer;
    int extensionPosition;
    double armPower;

    int armMode=1;

    public static final double intakeRotationThreshold = 35; //degrees

    @Override
    public void init() {
        super.init();
        armController = new Control.pid();
        Devices.rightBackDriveMotor.setDirection(FORWARD);
        Devices.rightFrontDriveMotor.setDirection(FORWARD);
        leftBackDriveMotor.setDirection(REVERSE);
        leftFrontDriveMotor.setDirection(REVERSE);
        Encoders.resetMotorEnc(slideLiftMotor);
        Encoders.resetMotorEnc(armLiftMotor1);
        boxMover.setPosition(0.35);
        dumpTimer = new ElapsedTime();
        slowModeCoolDown = new ElapsedTime();
        armModeCoolDown = new ElapsedTime();
        intakeTimer = new ElapsedTime();
        armPower=0;
        extensionPosition=0;
        Devices.slideLiftMotor.setDirection(REVERSE);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        //super.loop();

        // drive using tankanum
        if (gamepad2.touchpad && slowModeCoolDown.seconds()>0.5) {
            slowMode = !slowMode;
            slowModeCoolDown.reset();
        }
        if (slowMode) Control.drive.tankDrive(gamepad2.left_stick_y/5, gamepad2.right_stick_y/5);
        else Control.drive.tankDrive(gamepad2.left_stick_y, gamepad2.right_stick_y);


        currentAngle = Control.conversion.armEncoderToAngle(armLiftMotor1.getCurrentPosition());


        //intake
        if (gamepad1.right_trigger>0.5) {
            intakeMotor.setPower(1);
        } else if (gamepad1.left_trigger>0.5) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }

        //control arm extension

        if (gamepad1.right_bumper&& slideLiftMotor.getCurrentPosition()<900) {
            slideLiftMotor.setPower(0.2);
            extensionPosition = slideLiftMotor.getCurrentPosition();
        }
        else if (gamepad1.left_bumper && slideLiftMotor.getCurrentPosition()>0) {
            slideLiftMotor.setPower(0.2);
            extensionPosition = slideLiftMotor.getCurrentPosition();
        }
        else {
            //Control.motor.linearSlideSetPosition(slideLiftMotor, extensionPosition);
            if (slideLiftMotor.getCurrentPosition()>extensionPosition+10) slideLiftMotor.setPower(0.2);
            else if (slideLiftMotor.getCurrentPosition()<extensionPosition-10) slideLiftMotor.setPower(-0.2);
            else slideLiftMotor.setPower(0);
        }

        // arm rotation mode switching

        if (armModeCoolDown.seconds()>.25) {
            if (gamepad1.dpad_up && armMode < 2) {
                if (armMode==3) armMode=0;
                else armMode++;
                armModeCoolDown.reset();
                updateSlide();
            } else if (gamepad1.dpad_down && armMode > 0) {
                if (armMode == 3)
                    armMode = 0;
                armMode--;
                armModeCoolDown.reset();
                if (armMode==1) {
                    intakeMotor.setPower(1);
                    intakeTimer.reset();
                }
                updateSlide();
            } else if (gamepad1.dpad_right) {
                armMode = 3;
                armModeCoolDown.reset();
                updateSlide();
            }
        }

        //arm rotation control

        if (armMode==0) {
            //intake
            armPower = -0.03;
            boxMover.setPosition(0.4);
            // armController.rotateWithPid(-8, currentAngle, .001, K_I, 0.01);
        } else if (armMode==1) {
            //default
            if (currentAngle<-5) {
                armPower = 0.1;
                        //armController.rotateWithPid(0, currentAngle, .01, K_I, K_D);
            }  else if (currentAngle<5) {
                armPower=0;
            } else if (currentAngle>80) {
                armPower=-0.17;
            } else {
                armPower=0.025;
            }

        } else if (armMode==2) {
            //outtake
            //armPower = armController.rotateWithPid(90, (currentAngle), .01, K_I, K_D);
            if (currentAngle<90)
                armPower = 0.2*Math.pow((90-currentAngle)/90, 2);
            else
                armPower = 0;
        } else if (armMode==3) {
            //shared hub
            armPower = armController.rotateWithPid(5, currentAngle, .01, K_I, K_D);
        } else {
            armPower = 0;
        }
        Control.motor.moveMotor(armLiftMotor1, armPower);


        //dumping
        if (gamepad1.square){
            dumping = true;
            dumpTimer.reset();
        }

        if (dumpTimer.seconds()>1 && dumpTimer.seconds()<2) {
            stopDumping();
        }

        if (dumping) {
            //intake.setPower(1);
            if (currentAngle<intakeRotationThreshold) {
                boxMover.setPosition(1);
            }
            else
                boxMover.setPosition(.35);
        } else {
            if (currentAngle<0) {
                boxMover.setPosition(0.45);
            }
            else if (currentAngle<intakeRotationThreshold) {
                boxMover.setPosition(.35);
            }
            else {
                boxMover.setPosition(1);
            }

        }

        //control spinner
        if (gamepad1.cross)
            spinner.setPower(0.35);
            //Control.motor.moveMotor(spinner, 0.35); //negative if red
        else if (gamepad1.triangle)
            spinner.setPower(-0.35);
            //Control.motor.moveMotor(spinner, -0.35);
        else
            spinner.setPower(0);
            //Control.motor.moveMotor(spinner, 0);

        if (intakeTimer.seconds()>1)
            intakeMotor.setPower(0);

        //telemetry

        //telemetry.addData("arm enconder reading 1: ", armLiftMotor1.getCurrentPosition());
        telemetry.addData("arm extender encoder reading: ", slideLiftMotor.getCurrentPosition());
        telemetry.addData("current angle: ", currentAngle);
        //telemetry.addData("box mover: ", boxMover.getPosition());
        //telemetry.addData("set point: ", restPosition);
        //telemetry.addData("arm power: ", armPower);
        telemetry.addData("extension position", extensionPosition);
        telemetry.addData("arm mode: ", armMode);
        //telemetry.addData("box mover: ", boxMover.getPosition());
    }

    public void stopDumping() {
        dumping = false;
        if (currentAngle>50)
            intakeMotor.setPower(1);
        armMode=1;
    }

    public void updateSlide() {
        if (armMode==0) {
            extensionPosition=700;
        } else if (armMode==1) {
            //if (currentAngle<-5) extensionPosition = 400;
            //else
                extensionPosition=0;
        } else if (armMode==2) {
            extensionPosition=450;
        } else if (armMode==3) {
            extensionPosition=500;
        }


        if (armMode==1) boxMover.setPosition(1);
        if (armMode==0) boxMover.setPosition(0.35);
    }
}