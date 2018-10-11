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

    boolean flipUp, flipControl = false;
    double boxPos=1;
    boolean boxUpControl, boxDownControl = false;
    boolean  intakeControl =false;
    boolean intakeUp = true;

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
        // toggle buttons
        if(gamepad2.a){
            flipControl = true;
        }

        if(!gamepad2.a && flipControl) {
            if (flipUp) {
                robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() - 400);
                robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() - 400);
                robot.flipL.setPower(-0.1);
                robot.flipR.setPower(-0.1);
                flipControl = false;
                flipUp = false;
            } else {
                robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() + 400);
                robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() + 400);
                robot.flipL.setPower(0.1);
                robot.flipR.setPower(0.1);
                flipControl = false;
                flipUp = true;
            }
        }

        if(gamepad2.x){
            intakeControl = true;
        }

        if(!gamepad2.x && intakeControl){
            intakeControl = false;
            intakeUp = !intakeUp;
        }

        if(intakeUp){
            robot.intakeServo.setPosition(0.3);
        } else {
            robot.intakeServo.setPosition(1.0);
        }

        if(gamepad2.dpad_up){
            boxUpControl=true;
        }

        if(!gamepad2.dpad_up && boxUpControl){
            if(boxPos > 0.1) {
                boxPos -= 0.1;
            }
            boxUpControl = false;
        }

        if(gamepad2.dpad_down){
            boxDownControl = true;
        }

        if(!gamepad2.dpad_down && boxDownControl){
            if(boxPos < 1) {
                boxPos += 0.1;
            }
            boxDownControl = false;
        }
        robot.boxR.setPosition(boxPos);
        robot.boxL.setPosition(1.06- boxPos);


        if(gamepad2.left_stick_y > 0.05){
            robot.intake.setPower(-0.5);
        } else if(gamepad2.left_stick_y < -0.05){
            robot.intake.setPower(0.5);
        } else {
            robot.intake.setPower(0);
        }
        double lx = scaleInput(gamepad1.left_stick_x);
        double rx = scaleInput(gamepad1.left_stick_x);
        double ly = scaleInput(-gamepad1.left_stick_y);
        double ry = scaleInput(-gamepad1.right_stick_y);
        driveArcade(ry, lx);


        telemetry.addData("boxPos",boxPos);
        telemetry.update();
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
