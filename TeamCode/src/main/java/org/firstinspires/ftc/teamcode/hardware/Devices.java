package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class Devices {
    // to add a hardware device, initialize the device here and map them in BaseBot
    public static DcMotor leftFrontDriveMotor, rightFrontDriveMotor, leftBackDriveMotor, rightBackDriveMotor, armLiftMotor1, armLiftMotor2, slideLiftMotor, spinner;

    public static Servo boxMover;//, support;
    public static CRServo intake;
    public static RevBlinkinLedDriver lightStrip;
    public static DistanceSensor distanceSensor;
    public static BNO055IMU imu;
    public static WebcamName webcam;

    // NOTE: deviceName should be the same as the name specified on the configuration
    public static void initDevices(HardwareMap hardwareMap) {
        // comment out the drive and imu initialization if you plan on using roadrunner
        Devices.leftBackDriveMotor = hardwareMap.get(DcMotor.class, "backLeft");
        Devices.rightBackDriveMotor = hardwareMap.get(DcMotor.class, "backRight");
        Devices.leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        Devices.rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "frontRight");
        Devices.armLiftMotor1 = hardwareMap.get(DcMotor.class, "armLift1");
        Devices.armLiftMotor2 = hardwareMap.get(DcMotor.class, "armLift2");

        Devices.slideLiftMotor = hardwareMap.get(DcMotor.class, "slide");
        Devices.spinner = hardwareMap.get(DcMotor.class, "spinner");

        Devices.intake = hardwareMap.get(CRServo.class, "intake");
        Devices.boxMover = hardwareMap.get(Servo.class, "boxMover");
        //Devices.support = hardwareMap.get(Servo.class, "support");
        Control.drive.configureDriveMotors();

        Devices.imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);
        Devices.webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
/*
        rightFrontDriveMotor.setDirection(FORWARD);
        rightBackDriveMotor.setDirection(FORWARD);
        leftFrontDriveMotor.setDirection(REVERSE);
        leftBackDriveMotor.setDirection(REVERSE);


 */




//        Devices.armLiftMotor = hardwareMap.get(DcMotor.class, "armLiftMotor");
        Devices.armLiftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Devices.armLiftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Devices.armAdjustServo = hardwareMap.get(Servo.class,"armAdjustServo");

//        Devices.lightStrip = hardwareMap.get(RevBlinkinLedDriver.class, "lightStrip");
//        Devices.distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }
}
