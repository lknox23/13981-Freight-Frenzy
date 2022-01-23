package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.*;

import java.util.List;

@Disabled
@Autonomous

public class SampleAutoGyro extends LinearOpMode {

    public void runOpMode() {
        Devices.initDevices(hardwareMap);
        Control.sensor.initGyro();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Control.auto.strafeToPosition(10, 0.5);

        Control.auto.turnWithGyro(90, -1.0);

        Control.auto.moveWithEncoder(10, 0.5);
    }
}
