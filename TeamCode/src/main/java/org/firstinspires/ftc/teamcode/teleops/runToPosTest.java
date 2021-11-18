package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor2;

@TeleOp
public class runToPosTest extends LinearOpMode {
    double power=0.2;
    int targetPosition= (int)Control.auto.armAngleToEncoder(15);

    @Override
    public void runOpMode() {
        Devices.initDevices(hardwareMap);
        armLiftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLiftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (Encoders.getMotorEnc(armLiftMotor1) > targetPosition)
            power *= -1.0;

            armLiftMotor1.setTargetPosition(targetPosition);
            armLiftMotor2.setTargetPosition(targetPosition);

            armLiftMotor1.setPower(power);
            armLiftMotor2.setPower(power);

            while (armLiftMotor1.isBusy() || armLiftMotor2.isBusy()) { }

            armLiftMotor1.setPower(0);
            armLiftMotor2.setPower(0);
    }

}
