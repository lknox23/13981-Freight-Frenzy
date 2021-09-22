package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.*;

/**
 * Template created by Ayaan Govil on 8/21/2021
 *
 * FTC Java Documentation: http://ftctechnh.github.io/ftc_app/doc/javadoc/index.html
 */

// basebot serves as the first execution point before flowing into the teleop/auto

public class BaseRobot extends OpMode {
    public ElapsedTime timer = new ElapsedTime();

    // this function runs when you hit the start button on the app
    @Override
    public void init() {
        // map the devices initialized in the Devices class
        // NOTE: deviceName should be the same as the name specified on the configuration
        Devices.leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBackDriveMotor");
        Devices.rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBackDriveMotor");
        Devices.leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
        Devices.rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDriveMotor");

        Devices.armLiftMotor = hardwareMap.get(DcMotor.class, "armLiftMotor");
        Devices.armClampServo = hardwareMap.get(Servo.class,"armClampServo");
    }

    // this function runs when you hit init after the start button
    @Override
    public void start() {
        timer.reset();
        Encoders.resetDriveEncs();
        Encoders.resetMotorEnc(Devices.armLiftMotor);
    }

    // this function runs when you hit the stop button
    public void stop() {
        timer.reset();
        Encoders.resetDriveEncs();
        Encoders.resetMotorEnc(Devices.armLiftMotor);
    }

    // this function loops while the bot is running
    @Override
    public void loop() {
        // telemetry (inherited from OpMode class) serves as logging on the phone - we're constantly tracking the drive motor encoders by doing this
        telemetry.addData("D00 Left Front Drive Motor Enc: ", Encoders.getMotorEnc(Devices.leftFrontDriveMotor));
        telemetry.addData("D01 Right Front Drive Motor Enc: ", Encoders.getMotorEnc(Devices.rightFrontDriveMotor));
        telemetry.addData("D02 Left Back Drive Motor Enc: ", Encoders.getMotorEnc(Devices.leftBackDriveMotor));
        telemetry.addData("D03 Right Back Drive Motor Enc: ", Encoders.getMotorEnc(Devices.rightBackDriveMotor));
    }
}
