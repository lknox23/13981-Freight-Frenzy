package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Devices;

import static org.firstinspires.ftc.teamcode.hardware.Devices.slideLiftMotor;

@TeleOp
public class diagnostics extends OpMode {
    public void init() {
        Devices.initDevices(hardwareMap);
    }
    public void start() {}
    public void loop() {
        telemetry.addData("arm extension: ", slideLiftMotor.getCurrentPosition());

    }

}
