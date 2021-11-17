package org.firstinspires.ftc.teamcode.autos;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

import static org.firstinspires.ftc.teamcode.hardware.Devices.spinner;

@Autonomous
public class RedPark extends LinearOpMode {
    public void runOpMode() {


        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(40)
                .build();

        Devices.initDevices(hardwareMap);

        drive.setPoseEstimate(new
                Pose2d(10, 10, Math.toRadians(90)));

        waitForStart();

        if (isStopRequested())
            return;

        drive.followTrajectory(traj1);
    }
}
