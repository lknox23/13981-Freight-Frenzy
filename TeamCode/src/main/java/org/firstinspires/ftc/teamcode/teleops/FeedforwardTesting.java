package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;
import org.firstinspires.ftc.teamcode.hardware.FeedforwardController;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor2;

@Config
@TeleOp

public class FeedforwardTesting extends BaseRobot {

    public static double maxSpeed = 435.0/60*360/2;
    public static double maxAcceleration = 21;
    //public static double f = 1/maxSpeed;
    public static double v = 0;
    public static double a = 0;


    double output=0;
    FeedforwardController ff;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        super.init();
        ff = new FeedforwardController(v, a, maxSpeed, maxAcceleration, false);
    }

    @Override
    public void start() { super.start(); }

    @Override
    public void loop() {
        super.loop();
        output = ff.getOutput(armLiftMotor1.getCurrentPosition(), 30);
        if (gamepad1.dpad_up) {
            Control.motor.moveMotor(armLiftMotor1, output);
        } else {
            Control.motor.moveMotor(armLiftMotor1, 0);
        }
        Control.drive.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);

        ff.addTelemetry(telemetry);
        telemetry.addData("output: ", output);
        telemetry.update();
    }
}