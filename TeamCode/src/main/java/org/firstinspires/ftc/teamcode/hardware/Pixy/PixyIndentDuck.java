package org.firstinspires.ftc.teamcode.hardware.Pixy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.hardware.ConstantVariables;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

@Autonomous
public class PixyIndentDuck extends OpMode {
    PixyCam pixyCam;
    PixyBlock duck;
    private int location; //must be out of 3 | -1 if not found
    ElapsedTime elapsedTime = new ElapsedTime();


    @Override
    public void init() {
        pixyCam = hardwareMap.get(PixyCam.class, "pixycam"); //init pixy cam
        location = -1;

    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     * this gets the blocklist for the largest block on screen
     */
    @Override
    public void loop() {
        duck = pixyCam.getBiggestBlock(1); //duck is the biggest block pixy cam found for signature 1 (as trained)
        if (duck.isEmpty()) { //check to see if pixy found a block
            location = -1; //returns -1 if no duck found
        } else {
            location = getLocation(duck.x);
        }

        telemetry.addData("DuckLocation | -1 = Not found", location);
        telemetry.update();
    }

    public int getLocation(int xCord) {
        if (xCord < ConstantVariables.PIXY_MAX_X / 3)
            return 1;
        else if (xCord >= ConstantVariables.PIXY_MAX_X / 3 && xCord < ((ConstantVariables.PIXY_MAX_X / 3) * 2)) {
            return 2;
        } else if (xCord >= ((ConstantVariables.PIXY_MAX_X/3) * 2) && xCord < ConstantVariables.PIXY_MAX_X) {
            return 3;
        } else {
            return -1; //if it cant figure it out just return -1
        }
    }
}