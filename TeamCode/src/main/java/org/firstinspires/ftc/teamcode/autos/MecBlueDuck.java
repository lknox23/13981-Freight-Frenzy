package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

import static org.firstinspires.ftc.teamcode.hardware.Devices.spinner;

@Autonomous
public class MecBlueDuck extends LinearOpMode {

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-11, -47), Math.toRadians(90))
                //above .splineTo needs tuning
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .back(10)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .forward(100)
                .build();
        Devices.initDevices(hardwareMap);

        drive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));


        waitForStart();

        if (isStopRequested()) return;

        //go to carousel
        Control.auto.moveWithEncoder(10, 0.5);
        //drive.followTrajectory(traj1);
        //Control.auto.turnWithGyro(10, 1);
        //Control.auto.moveWithEncoder(-5, 0.5);

        //spin carousel
        Control.auto.spinCarousel(spinner, .35);
        telemetry.addLine("carousel turned");
        telemetry.update();

        //wait until end of auto
        sleep(2000);

        //park in warehouse
        Control.auto.moveWithEncoder(-43, 0.5);
    }
}
