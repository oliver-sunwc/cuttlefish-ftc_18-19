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
    boolean x;
    boolean y;
    boolean a;
    boolean b;
    double servoPos = 0.9;
    double dumpPos = 0.2;
    double downDump = 0.2;
    double upDump = 0.075;
    double midDump = 0.08;
    double downArm = 0.9;
    double upArm = 0.10;


    @Override
    public void init() {
        robot.init(hardwareMap,false);

        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        robot.flipLArm.setPosition(servoPos);
        robot.flipRArm.setPosition(1-servoPos);
        robot.dumpFlip.setPosition(dumpPos);
        telemetry.addData("servoPos", robot.flipLArm.getPosition());
        telemetry.addData("dumpPos", robot.dumpFlip.getPosition());

        if(!a && gamepad2.a) {
            servoPos += 0.05;
        }
        a = gamepad2.a;

        if(!b && gamepad2.b) {
            servoPos -= 0.05;
        }
        b = gamepad2.b;

        if(!x && gamepad2.x) {
            dumpPos += 0.05;
        }
        x = gamepad2.x;

        if(!y && gamepad2.y) {
            dumpPos -= 0.05;
        }
        y = gamepad2.y;
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
