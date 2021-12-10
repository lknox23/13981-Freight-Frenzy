package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor2;
import static org.firstinspires.ftc.teamcode.hardware.Devices.slideLiftMotor;

@Config
@TeleOp
public class runToPosTest extends LinearOpMode {
    public static double p = .02;
    public static double i = 0;
    public static double d = 0.1;
    double power=0.2;
    double currentAngle;
    int targetPosition= (int)Control.auto.armAngleToEncoder(45);

    double holdPosition;

    Control.pid armController;


    @Override
    public void runOpMode() {
        armController  = new Control.pid();

        Devices.initDevices(hardwareMap);
        armLiftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        goToPos((int)Control.auto.armAngleToEncoder(45), armLiftMotor1.getCurrentPosition());
        holdPos();
    }



    public void goToPos(double goal, double current) {
        double power;
        if (current>goal) {
            if (currentAngle<60) power = -0.01;
            else if (currentAngle<90)power = -0.1;
            else power = -0.2;
        } else {
            power = 0.2;

            armLiftMotor1.setTargetPosition(targetPosition);
            armLiftMotor2.setTargetPosition(targetPosition);
            armLiftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLiftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLiftMotor1.setPower(power);
            armLiftMotor2.setPower(power);
        }


        while (armLiftMotor1.isBusy() || armLiftMotor2.isBusy()) { }

        armLiftMotor1.setPower(0);
        armLiftMotor2.setPower(0);
    }

    public void holdPos() {
        holdPosition=Control.auto.armEncoderToAngle(armLiftMotor1.getCurrentPosition());
        telemetry.addData("hold position", holdPosition);
        telemetry.update();
        while (!isStopRequested()) {
            power = armController.rotateWithPid(holdPosition, Control.auto.armEncoderToAngle(armLiftMotor1.getCurrentPosition()), p, i, d);
            armLiftMotor1.setPower(power);
            armLiftMotor2.setPower(power);
            telemetry.addData("power: ", power);
            telemetry.addData("angle", Control.auto.armEncoderToAngle(armLiftMotor1.getCurrentPosition()));
            telemetry.update();
        }
    }

}
