package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

//8 notches up on carousel side
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.hardware.Devices.spinner;
@Autonomous
public class RedDuckAuto extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleTankDrive(hardwareMap);
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .build();
        //turn -60
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .back(10)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();
        //turn -75
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d())
                .forward(100)
                .build();
        Devices.initDevices(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-35, -60, Math.toRadians(135)));

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
        Control.auto.turnWithGyro(75, 1);
        Control.auto.moveWithEncoder(-10, 0.2);
        Control.auto.spinCarousel(spinner, -.35);
        Control.auto.moveWithEncoder(5, .3);
        sleep(2000);
        Control.auto.turnWithGyro(37, 1);
        Control.auto.moveWithEncoder(45, 1);
    }
}
