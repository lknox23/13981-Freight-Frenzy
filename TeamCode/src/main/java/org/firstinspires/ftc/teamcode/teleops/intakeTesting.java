package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@TeleOp

public class intakeTesting extends BaseRobot {
DcMotor intakeMotor;
    @Override
    public void init() {
        //super.init();
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

    }

    @Override
    public void start() { super.start(); }

    @Override
    public void loop() {
        super.loop();

        if(gamepad1.a){
            intakeMotor.setPower(1);
        }
        else if(gamepad1.b){
            intakeMotor.setPower(-1);
        }
        else{
            intakeMotor.setPower(0);
        }

    }
}