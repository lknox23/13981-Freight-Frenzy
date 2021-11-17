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
public class BlueDuckAuto extends LinearOpMode {

    public void runOpMode() {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .back(15)
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

        drive.followTrajectory(traj1);
        Control.auto.turnWithGyro(10, 1);

        Control.auto.moveWithEncoder(-5, 0.5);
        Control.auto.spinCarousel(spinner, .35);
        telemetry.addLine("carousel turned");
        telemetry.update();
        sleep(2000);

        Control.auto.moveWithEncoder(43, 1);
        /*
        Encoders.driveResetEncs();
        waitForStart();
        moveWithEncoder(-10, 0.7);

        telemetry.addData("left front power: ", leftFrontDriveMotor.getTargetPosition());

         */
}
}
