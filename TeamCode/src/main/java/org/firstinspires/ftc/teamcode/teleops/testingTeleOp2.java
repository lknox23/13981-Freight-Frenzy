package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.boxMover;
import static org.firstinspires.ftc.teamcode.hardware.Devices.intakeMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.leftBackDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.leftFrontDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.outtake;
import static org.firstinspires.ftc.teamcode.hardware.Devices.rightBackDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.rightFrontDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.slideLiftMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.spinner;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

@TeleOp
public class testingTeleOp2 extends BaseRobot {


    //double currentAngle;
    //double restPosition=0;
    Control.pid armController;
    boolean slowMode=false;
    //boolean dumping = false;
    ElapsedTime dumpTimer;
    ElapsedTime slowModeCoolDown;
    ElapsedTime armModeCoolDown;
    ElapsedTime intakeTimer;
    int extensionPosition;
    int anglePosition;
    double outtakePosition=0.5;
    double outtakeLeverPosition = 0.5;
    double armPower;
    int state=0;

    int armMode=1;

    public static final double intakeRotationThreshold = 35; //degrees

    @Override
    public void init() {
        super.init();
        armController = new Control.pid();
        Devices.rightBackDriveMotor.setDirection(REVERSE);
        Devices.rightFrontDriveMotor.setDirection(REVERSE);
        leftBackDriveMotor.setDirection(REVERSE);
        leftFrontDriveMotor.setDirection(FORWARD);
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
        if (gamepad2.touchpad && slowModeCoolDown.seconds()>0.5) {
            slowMode = !slowMode;
            slowModeCoolDown.reset();
        }
        if (slowMode) Control.drive.tankanumDrive(gamepad1.right_stick_y/5, gamepad1.left_stick_y/5, gamepad1.left_stick_x/5);
        else Control.drive.tankanumDrive(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.left_stick_x);

        if (state==0) { //start
            anglePosition=0;
            //extensionPosition = 0;
            outtakePosition=0.32;
            outtakeLeverPosition = 0.2;
        } else if (state==1) { //shared hub
            anglePosition = 0;
            //extensionPosition = 850;
            outtakePosition = 0.32;
            outtakeLeverPosition = 0.5;
        } else if (state==2) { //shared hub dropping
            anglePosition = 0;
            //extensionPosition = 850;
            outtakePosition = 0.32;
            outtakeLeverPosition = 1;
        } else if (state==3) { //alliance hub
            anglePosition=275;
            //extensionPosition = 850;
            outtakePosition = 0.15;
            outtakeLeverPosition=0.5;
        } else if (state==4) { //alliance hub dropping
            anglePosition=275;
            //extensionPosition = 850;
            outtakePosition = 0.15;
            outtakeLeverPosition=1;
        }
        //arm control: arm moves up and down
        if(gamepad1.dpad_down){
            //armPower = -0.1;
            //anglePosition = armLiftMotor1.getCurrentPosition();
            state = 0;
        }
        else if (gamepad1.dpad_left) {
            state = 1;
        }
        else if(gamepad1.dpad_up){
            //armPower = 0.1;
            //anglePosition = armLiftMotor1.getCurrentPosition();
            state = 3;
        }
        else{
            if (armLiftMotor1.getCurrentPosition()<anglePosition-10)
                armPower = 0.2; //.15
            else if (armLiftMotor1.getCurrentPosition()>anglePosition+10)
                armPower = -0.2;
            else if (armLiftMotor1.getCurrentPosition()>20 && armLiftMotor1.getCurrentPosition()<70)
                armPower = .01;
            else
                armPower = 0;
        }
        Control.motor.moveMotor(armLiftMotor1, armPower);

        //arm control: arm extends and retracts
        if (gamepad1.dpad_down) {
            extensionPosition = 0;
        } else if (gamepad1.dpad_left) {
            extensionPosition = 0;
        } else if (gamepad1.dpad_up) {
            extensionPosition = 850;
        }
        if (gamepad1.right_bumper&& slideLiftMotor.getCurrentPosition()<900) {
            slideLiftMotor.setPower(0.2);
            extensionPosition = slideLiftMotor.getCurrentPosition();
        }
        else if (gamepad1.left_bumper && slideLiftMotor.getCurrentPosition()>-20) {
            slideLiftMotor.setPower(-0.2);
            extensionPosition = slideLiftMotor.getCurrentPosition();
        }
        else {
            if (slideLiftMotor.getCurrentPosition()>extensionPosition+20)
                slideLiftMotor.setPower(-0.2);
            else if (slideLiftMotor.getCurrentPosition()<extensionPosition-20)
                slideLiftMotor.setPower(0.2);
            else slideLiftMotor.setPower(0);
        }

        //intake
        if (gamepad1.right_trigger>0.5) {
            intakeMotor.setPower(1);
        } else if (gamepad1.left_trigger>0.5) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }

        //outtake box
        if (gamepad1.circle) {
            if (state==1 || state == 3) {
                state++;
                dumpTimer.reset();
            }
        }
        if (dumpTimer.seconds()>1 && (state==2 || state==4)) {
            state--;
        }
        boxMover.setPosition(outtakePosition);

        //outtake lever
        outtake.setPosition(outtakeLeverPosition);

        //spinner
        if (gamepad1.cross) {
            spinner.setPower(-1);
        } else
            spinner.setPower(0);

        telemetry.addData("slide position: ", slideLiftMotor.getCurrentPosition());
        telemetry.addData("actuator position: ", armLiftMotor1.getCurrentPosition());
        telemetry.addData("actuator angle: ", Control.conversion.armEncoderToAngle(armLiftMotor1.getCurrentPosition()));
    }




}
