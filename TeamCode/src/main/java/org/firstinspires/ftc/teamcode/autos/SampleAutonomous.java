package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.BaseRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.hardware.*;

@Autonomous

// basic autonomous that moves forward 10 inches, waits 5 seconds, turns around, then drives back

public class SampleAutonomous extends BaseRobot {
    private int stage = 0;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        // autonomous uses a switch statement to step through each part of the autonomous
        switch (stage) {
            case 0:
                timer.reset();
                Control.drive(1, 10);
                if (timer.seconds() >= 5) {
                    stage++;
                }
                break;

            case 1:
                if (Control.turn(1.0, 180)) {
                    stage++;
                }
                break;

            case 2:
                timer.reset();
                Control.drive(1, 10);
                stage++;
                break;

            default:
                break;
        }
    }
}