package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;
import android.provider.MediaStore;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

/**
 * Manual with Arcade Drive
 */
@TeleOp(name = "teleOpTEST", group = "Rover")
public class teleOpTEST extends OpMode {

    roverHMAP robot = new roverHMAP();
    boolean x1;
    boolean y1;
    boolean a1;
    boolean b1;
    boolean x2;
    boolean y2;
    boolean a2;
    boolean b2;
    boolean flipDown = true;
    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    boolean dumpTrigger = false;
    boolean dumpTrigger2 = false;
    double servoPos = 0.5;
    double dumpPos = 0.5;
    double rotatePos = 0.5;
    double upDump = 0.43;

    @Override
    public void init() {
        robot.init(hardwareMap,false);
        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rotateArm.setPosition(0.5);
        robot.flipLArm.setPosition(1);
        robot.flipRArm.setPosition(0);
        robot.dumpFlip.setPosition(0.43);
        timer1.startTime();
        timer2.startTime();
    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        telemetry.addData("servoPos", robot.flipLArm.getPosition());
        telemetry.addData("dumpPos", robot.dumpFlip.getPosition());
        telemetry.addData("rotatePos", robot.rotateArm.getPosition());
        telemetry.update();

        robot.rotateArm.setPosition(rotatePos);

        if(!a1 && gamepad1.a) {
            upDump = servoPos;
        }
        a1 = gamepad1.a;

        if(!b1 && gamepad1.b) {
            rotatePos -= 0.05;
        }
        b1 = gamepad1.b;

        if(!x1 && gamepad1.x) {
            if(flipDown){
                flipDown = false;
                robot.flipLArm.setPosition(0.5);
                robot.flipRArm.setPosition(0.5);
                robot.dumpFlip.setPosition(0.7);
                dumpTrigger = true;
                timer1.reset();
            } else {
                flipDown = true;
                robot.flipLArm.setPosition(1);
                robot.flipRArm.setPosition(0);
                robot.dumpFlip.setPosition(0.43);
            }
        }
        x1 = gamepad1.x;

        if(dumpTrigger && timer1.seconds() > 0.7){
            robot.flipLArm.setPosition(0.1);
            robot.flipRArm.setPosition(0.9);
            robot.dumpFlip.setPosition(0.1);
            dumpTrigger = false;
        }

        if(!y1 && gamepad1.y) {
            rotatePos += 0.05;
        }
        y1 = gamepad1.y;

        if(!a2 && gamepad2.a) {
            servoPos += 0.05;
        }
        a2 = gamepad2.a;

        if(!b2 && gamepad2.b) {
            servoPos -= 0.05;
        }
        b2 = gamepad2.b;

        if(!x2 && gamepad2.x) {
            dumpPos += 0.05;
        }
        x2 = gamepad2.x;

        if(!y2 && gamepad2.y) {
            dumpPos -= 0.05;
        }
        y2 = gamepad2.y;
    }



    /*public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }*/

    /*
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return*/

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    /*public double getHeading() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Double.parseDouble(formatAngle(robot.angles.angleUnit, robot.angles.firstAngle));
    }*/
}
