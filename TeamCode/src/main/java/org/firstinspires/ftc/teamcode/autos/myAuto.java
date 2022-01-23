//OBSOLETE
package org.firstinspires.ftc.teamcode.autos;


import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_D;
import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_I;
import static org.firstinspires.ftc.teamcode.hardware.ConstantVariables.K_P;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.boxMover;
import static org.firstinspires.ftc.teamcode.hardware.Devices.slideLiftMotor;
import static org.firstinspires.ftc.teamcode.hardware.OpenCv.initWebcam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;
import org.firstinspires.ftc.teamcode.hardware.OpenCv;
import org.firstinspires.ftc.teamcode.hardware.SamplePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Disabled
public class myAuto extends LinearOpMode {



    /*
     * Steps:
     * 1. Deliver preloaded box to alliance shipping hub
     * 2. Ducks
     * 3. Park
     * */


    SampleMecanumDrive drive;
    SamplePipeline pipeline;
    OpenCvWebcam webcam;

    double coeff=1;
    Trajectory traj1, traj2, traj3;

    int duckPositionIndex;

    @Override
    public void runOpMode() {
        initializeStuff();
        initRR();
        telemetry.addLine("initialized");

        waitForStart();

        detectDucks();
        telemetry.addData("duck position: ", duckPositionIndex);
        telemetry.update();

        if (isStopRequested()) return;

        goToShippingHub();
        telemetry.addLine("shipping hub reached");
        telemetry.update();
        placePreloaded();
        telemetry.addLine("element placed");
        telemetry.update();
        goToCarousel();
        telemetry.addLine("carousel reached");
        telemetry.update();
        deliverDuck();
        telemetry.addLine("duck delivered");
        telemetry.update();
        park();
    }

    public void initializeStuff() {
        Devices.initDevices(hardwareMap);

        pipeline = new SamplePipeline();

        //OPENCV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });

        Control.sensor.initGyro();
        boxMover.setPosition(0.35);

    }

    public void initRR() {
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPosition = new Pose2d(10*coeff, 10*coeff, Math.toRadians(90));
        drive.setPoseEstimate(startingPosition);
        traj1 = drive.trajectoryBuilder(startingPosition)
                .splineTo(new Vector2d(-13, -40), Math.toRadians(90))
                .build();
        traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .splineTo(new Vector2d(-60, -60), Math.toRadians(180))
                .build();
        traj3 = drive.trajectoryBuilder(traj2.end(), false)
                .splineTo(new Vector2d(38, -45), Math.toRadians(0))
                .build();
    }

    public void detectDucks() {

        /*
        Recognition duck = Control.auto.getDuck(telemetry);

        double duckPositionAngle = duck.estimateAngleToObject(AngleUnit.DEGREES);
        duckPositionIndex = Control.auto.getDuckPositionIndexThree(duckPositionAngle);
         */


        if (pipeline.inRegion1())
            duckPositionIndex = 0;
        else if (pipeline.inRegion2())
            duckPositionIndex = 1;
        else
            duckPositionIndex = 3;
    }

    public void goToShippingHub(){
        drive.followTrajectory(traj1);
    }

    public void placePreloaded() {
        double currentAngle = Control.conversion.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
        double output;
        int angle;
        if (duckPositionIndex==0) angle = 0;
        else if (duckPositionIndex==1) angle = 30;
        else angle = 60;

        Control.pid armController = new Control.pid();

        boxMover.setPosition(1);

        while (opModeIsActive() && currentAngle<angle-5){
            currentAngle = Control.conversion.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
            output = armController.rotateWithPid(angle, (currentAngle), K_P, K_I, K_D);
            Control.motor.moveMotor(armLiftMotor1, output);
            if (slideLiftMotor.getCurrentPosition()<100) slideLiftMotor.setPower(0.2);
        }

        ElapsedTime timer = new ElapsedTime();
        telemetry.addLine("point reached");
        boxMover.setPosition(0);

        while (opModeIsActive()&&timer.seconds() < 1 ){
            currentAngle = Control.conversion.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor1));
            output = armController.rotateWithPid(angle, (currentAngle), K_P, K_I, K_D);
            Control.motor.moveMotor(armLiftMotor1, output);
            if (slideLiftMotor.getCurrentPosition()<100) slideLiftMotor.setPower(0.2);
        }
    }

    public void goToCarousel() {

        drive.followTrajectory(traj2);

    }

    public void deliverDuck() {
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
    }

    public void park(){
        drive.followTrajectory(traj3);
    }
}


