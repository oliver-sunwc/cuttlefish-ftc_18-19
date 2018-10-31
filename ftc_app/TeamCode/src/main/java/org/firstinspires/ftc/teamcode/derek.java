package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Manual with Arcade Drive
 */

@TeleOp(name = "DEREKDEREKDEREK", group = "Rover")
public class derek extends OpMode {
    public DcMotor m;


    @Override
    public void init(){
        m = hardwareMap.get(DcMotor.class, "m");

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        m.setPower(-gamepad1.left_stick_y);

    }



}
