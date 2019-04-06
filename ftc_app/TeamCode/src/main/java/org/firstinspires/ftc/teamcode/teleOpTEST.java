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
        robot.dumpFlip.setPosition(0);
        timer1.startTime();
        timer2.startTime();

        robot.dump.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dump.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        telemetry.addData("rotatePos", rotatePos);
        robot.rotateArm.setPosition(rotatePos);
        if(gamepad2.a){
            rotatePos += 0.05;
        }
        if(gamepad2.b){
            rotatePos -= 0.05;
        }
        if(gamepad2.y){
            rotatePos = 0.5;
        }

        telemetry.addData("ticks",robot.dump.getCurrentPosition());
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
