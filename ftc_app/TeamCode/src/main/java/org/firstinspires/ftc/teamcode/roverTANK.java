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

@TeleOp(name = "Tank", group = "Rover")
public class roverTANK extends OpMode {

    roverHMAP robot = new roverHMAP();


    @Override
    public void init(){
        robot.init(hardwareMap);
        boolean dirToggle = false;
    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){


    }

    void drive() {
        robot.fL.setPower(gamepad1.left_stick_y);
        robot.bL.setPower(gamepad1.left_stick_y);
        robot.fR.setPower(gamepad1.right_stick_y);
        robot.bR.setPower(gamepad1.right_stick_y);

    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
