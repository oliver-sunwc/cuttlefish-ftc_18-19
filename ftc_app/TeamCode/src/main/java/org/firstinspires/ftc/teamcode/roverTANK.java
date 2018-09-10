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
    boolean dirToggle = false;
    double lx, rx,ly,ry;

    @Override
    public void init(){
        robot.init(hardwareMap);
    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double ry = -gamepad1.right_stick_y;

    }

    void driveTank(double ly, double ry) {
        robot.fL.setPower(ly);
        robot.bL.setPower(ly);
        robot.fR.setPower(ry);
        robot.bR.setPower(ry);

    }

    void driveArcade(double ry, double lx){
        robot.fL.setPower(ry + lx);
        robot.bL.setPower(ry + lx);
        robot.fR.setPower(ry - lx);
        robot.bR.setPower(ry - lx);
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
