package org.firstinspires.ftc.teamcode.autos;


import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.spinner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;


@Disabled
public class BlueSideAuto extends LinearOpMode {

    /*
     * Steps:
     * 1. Deliver preloaded box to alliance shipping hub
     * 2. Ducks
     * 3. Park
     * */

    int duckPositionIndex;
    SampleTankDrive drive;
    double coeff=-1;
    @Override
    public void runOpMode() {
        initializeStuff();

        waitForStart();

    }

    public void initializeStuff() {
        Devices.initDevices(hardwareMap);
        Control.auto.initTF("FreightFrenzy_DM.tflite", new String[]{
                "Duck",
                "Marker"
        }, 1.0, hardwareMap);
        Control.sensor.initGyro();
        drive = new SampleTankDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(10*coeff, 10*coeff, Math.toRadians(-90)));
    }

    public void detectDucks() {
        Recognition duck = Control.auto.getDuck(telemetry);

        double duckPositionAngle = duck.estimateAngleToObject(AngleUnit.DEGREES);
        duckPositionIndex = Control.auto.getDuckPositionIndexThree(duckPositionAngle);
    }

    public void goToShippingHub(){
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-13, 40), Math.toRadians(-90))
                .build();
        drive.followTrajectory(traj1);
    }

    /*public void placePreloaded() {
        double currentAngle = Control.auto.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
        double output;

        Control.pid armController = new Control.pid();

        while (armController.getSpeed(90, currentAngle) > 5){
            currentAngle = Control.auto.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
            output = armController.rotateWithPid(90, (currentAngle));
            Control.motor.moveMotor(armLiftMotor1, output);
        }

        ElapsedTime timer = new ElapsedTime;
        while (ElapsedTime.seconds() < ){

        }

    }*/

    public void goToCarousel() {
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(new Vector2d(-60, 60), Math.toRadians(180))
                .build();
        drive.followTrajectory(traj2);

    }

    public void deliverDucks() {
        /*
        ElapsedTime runtime;
        runtime = new ElapsedTime();
        double oldTime;
        oldTime = 0;
        double dT = runtime.milliseconds() - oldTime;

        while (dT <= 2000) {
            dT = runtime.milliseconds() - oldTime;
            Control.motor.moveMotor(Devices.spinner, 0.5);
        }

         */
        Control.auto.spinCarousel(spinner, 0.5);
    }

    public void park(){
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(38, 45), Math.toRadians(0))
                .build();
        drive.followTrajectory(traj3);


    }
}
