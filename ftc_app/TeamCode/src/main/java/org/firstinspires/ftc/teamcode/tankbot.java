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

@TeleOp(name = "TankReal", group = "Rover")
public class tankbot extends OpMode {
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;

    @Override
    public void init(){
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        br.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        fl.setPower(-gamepad1.left_stick_y);
        bl.setPower(-gamepad1.left_stick_y);
        fr.setPower(-gamepad1.right_stick_y);
        br.setPower(-gamepad1.right_stick_y);
    }


}
