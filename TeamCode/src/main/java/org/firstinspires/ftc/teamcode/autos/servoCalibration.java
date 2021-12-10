package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Devices;

import static org.firstinspires.ftc.teamcode.hardware.Devices.boxMover;
import static org.firstinspires.ftc.teamcode.hardware.Devices.intake;

@Config
@Autonomous
public class servoCalibration extends LinearOpMode {
    public static double position=0;
    public void runOpMode() {
        Devices.initDevices(hardwareMap);
        waitForStart();
        while (!isStopRequested()) {
            boxMover.setPosition(position);
        }
    }
}
