package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Manual with Arcade Drive
 */

@TeleOp(name = "TankFinal", group = "Rover")
public class roverTeleOP extends OpMode {

    roverHMAP robot = new roverHMAP();
    boolean dirToggle = false;
    double lx, rx,ly,ry;

    @Override
    public void init(){
        robot.init(hardwareMap);
        robot.MArmL.setPosition(0.2);
        robot.MArmR.setPosition(0.7);
    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        double lx = scaleInput(gamepad1.left_stick_x);
        double rx = scaleInput(gamepad1.left_stick_x);
        double ly = scaleInput(-gamepad1.left_stick_y);
        double ry = scaleInput(-gamepad1.right_stick_y);
        driveArcade(ry, lx);
    }

    void driveTank(double ly, double ry) {
        robot.fl.setPower(ly);
        robot.bl.setPower(ly);
        robot.fr.setPower(ry);
        robot.br.setPower(ry);

        telemetry.addData("dl",robot.dMArmL.getDistance(DistanceUnit.CM));
        telemetry.addData("dl",robot.dMArmL.getDistance(DistanceUnit.CM));
        telemetry.update();

    }

    void driveArcade(double ry, double lx){
        robot.fl.setPower(ry + lx);
        robot.bl.setPower(ry + lx);
        robot.fr.setPower(ry - lx);
        robot.br.setPower(ry - lx);
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
