package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BaseRobot;

import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor1;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor2;

@TeleOp
public class maxAccelTest extends BaseRobot {
    double v=0;
    double oldV=0;
    double a=0;
    double oldPosition=0;
    double oldTime;
    ElapsedTime timer;

    double bestA=0;
    public void init() {
        super.init();
        timer = new ElapsedTime();
    }

    public void loop() {
        if (gamepad1.a) {
            armLiftMotor1.setPower(0.5);
            armLiftMotor2.setPower(0.5);
            v=(armLiftMotor1.getCurrentPosition()-oldPosition)/(timer.seconds()-oldTime);
            a=(v-oldV)/(timer.seconds()-oldTime);
            if (a>bestA)
                bestA=a;
        } else {
            armLiftMotor1.setPower(0);
            armLiftMotor2.setPower(0);
            telemetry.addData("max acceleration: ", bestA);
            telemetry.update();
        }
    }
}
