package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.boxMover;
import static org.firstinspires.ftc.teamcode.hardware.Devices.intakeMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.leftBackDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.leftFrontDriveMotor;
import static org.firstinspires.ftc.teamcode.hardware.Devices.slideLiftMotor;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

public class testingTeleOp2 extends BaseRobot {


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

        //arm control: arm moves up and down
        if(gamepad1.dpad_down){
            armPower = -0.5;
        }
        else if(gamepad1.dpad_up){
            armPower = 0.5;
        }
        else{
            armPower = 0;
        }
        Control.motor.moveMotor(armLiftMotor1, armPower);

        //intake
        if (gamepad1.right_trigger>0.5) {
            intakeMotor.setPower(1);
        } else if (gamepad1.left_trigger>0.5) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }

        if (gamepad1.b){
            double targetPosition = Control.conversion.armAngleToEncoder(30); //degrees
            Control.motor.setPositionInLoop(armLiftMotor1, 1, targetPosition);
        }

    }




}
