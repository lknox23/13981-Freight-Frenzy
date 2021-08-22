package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

public class Control extends Devices {


    // MOTORS


    // DC MOTOR

    // moves motor based on power
    // 1.0: forwards
    // 0.0: brake
    // -1.0 backwards
    public static void moveMotor(DcMotor motor, double power) {
        double speed = Range.clip(power, -1, 1);
        motor.setPower(speed);
    }

    // drive with each side correlating to gamepad sticks
    public static void tankDrive(double leftPwr, double rightPwr) {
        double leftPower = Range.clip(leftPwr, -1.0, 1.0);
        double rightPower = Range.clip(rightPwr, -1.0, 1.0);

        leftFrontDriveMotor.setPower(leftPower);
        leftBackDriveMotor.setPower(leftPower);
        rightFrontDriveMotor.setPower(-rightPower);
        rightBackDriveMotor.setPower(-rightPower);
    }

    // tankDrive + mecanum drive
    public static void tankanumDrive(double rightPwr, double leftPwr, double lateralPwr) {
        rightPwr *= -1;

        double leftFrontPower = Range.clip(leftPwr - lateralPwr, -1.0, 1.0);
        double leftBackPower = Range.clip(leftPwr + lateralPwr, -1.0, 1.0);
        double rightFrontPower = Range.clip(rightPwr - lateralPwr, -1.0, 1.0);
        double rightBackPower = Range.clip(rightPwr + lateralPwr, -1.0, 1.0);

        leftFrontDriveMotor.setPower(leftFrontPower);
        leftBackDriveMotor.setPower(leftBackPower);
        rightFrontDriveMotor.setPower(rightFrontPower);
        rightBackDriveMotor.setPower(rightBackPower);
    }

    public static boolean drive(double power, double inches) {
        double TARGET_ENC = ConstantVariables.K_PPIN_DRIVE * inches;
        double left_speed = -power;
        double right_speed = power;
        double error = -Encoders.getMotorEnc(Devices.leftFrontDriveMotor) - Encoders.getMotorEnc(Devices.rightFrontDriveMotor);

        error /= ConstantVariables.K_DRIVE_ERROR_P;
        left_speed += error;
        right_speed -= error;

        left_speed = Range.clip(left_speed, -1, 1);
        right_speed = Range.clip(right_speed, -1, 1);
        leftFrontDriveMotor.setPower(left_speed);
        leftBackDriveMotor.setPower(left_speed);
        rightFrontDriveMotor.setPower(right_speed);
        rightBackDriveMotor.setPower(right_speed);

        if (Math.abs(Encoders.getMotorEnc(Devices.rightFrontDriveMotor)) >= TARGET_ENC) {
            leftFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
            return true;
        } else return false;
    }

    // turns a specific amount of degrees
    // power: the speed to move (1.0 to -1.0)
    // degrees: the amount of degees to turn
    // returns whether it has reached the target degrees
    public static boolean turn(double power, double degrees) {
        double TARGET_ENC = Math.abs(ConstantVariables.K_PPDEG_DRIVE * degrees);

        double speed = Range.clip(power, -1, 1);
        leftFrontDriveMotor.setPower(-speed);
        leftBackDriveMotor.setPower(-speed);
        rightFrontDriveMotor.setPower(-speed);
        rightBackDriveMotor.setPower(-speed);

        if (Math.abs(Encoders.getMotorEnc(rightFrontDriveMotor)) >= TARGET_ENC) {
            leftFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
            return true;
        } else return false;
    }


    // SERVO


    // sets servo position based on pos
    // 1.0: highest position
    // 0.0: lowest position
    public static void setServoPosition(Servo servo, double pos) {
        double position = Range.clip(pos, 0, 1.0);
        servo.setPosition(position);
    }


    // SENSORS


    // COLOR SENSOR

    public static boolean checkBlackColor(int red, int blue) {
        return blue > (3.0/4)*red;
    }

    public static boolean checkGreenColor(int green, int blue, int red) {
        if (green > blue + red){
            return true;
        } else return false;
    }

    public static boolean checkBlueColor(int green, int blue, int red) {
        if (blue > green + red){
            return true;
        } else return false;
    }

    public static boolean checkRedColor(int green, int blue, int red) {
        if (red > blue + green){
            return true;
        } else return false;
    }
}
