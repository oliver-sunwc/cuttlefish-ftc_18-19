package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class roverAuto extends LinearOpMode {

    /*roverHMAP robot;
    public String telVar;
    public roverAuto(roverHMAP hwmap){
        robot = hwmap;
    } */
    public void runOpMode() throws InterruptedException{

    } /*
    //------------------------------------------------------------------------------------------------------------------------------
    //Driving Power Functions
    void stopDriving() {
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);
    }


    //distance=rate*duration duration=distance/rate
    //power drives forward, -power drives backward
    void verticalDrive(double power) {
        robot.fl.setPower(power);
        robot.fr.setPower(power);
        robot.bl.setPower(power);
        robot.br.setPower(power);
    }

    void verticalDrive2(double power, double angle){
        if(getHeading() - angle > 4){
            robot.fl.setPower(0.8*power);
            robot.fr.setPower(0.8*power);
            robot.bl.setPower(1.2*power);
            robot.br.setPower(1.2*power);
        } else if (angle - getHeading() > 4){
            robot.fl.setPower(1.2*power);
            robot.fr.setPower(1.2*power);
            robot.bl.setPower(0.8*power);
            robot.br.setPower(0.8*power);
        }else if(angle - getHeading() < 2 && angle - getHeading() > -2){
            robot.fl.setPower(power);
            robot.fr.setPower(power);
            robot.bl.setPower(power);
            robot.br.setPower(power);
        }
    }

    void rotateRight(double power) {
        robot.fl.setPower(-power);
        robot.bl.setPower(-power);
        robot.fr.setPower(power);
        robot.br.setPower(power);
    }

    void rotateLeft(double power) {
        rotateRight(-power);
    }


    //------------------------------------------------------------------------------------------------------------------------------
    //Encoder Functions


    void verticalDriveDistance(double power, double distance) throws InterruptedException {
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        distance = (int)(distance*robot.ticksPerInch);
        int flDist = robot.fl.getCurrentPosition();
        int frDist = robot.fr.getCurrentPosition();
        int blDist = robot.bl.getCurrentPosition();
        int brDist = robot.br.getCurrentPosition();
        verticalDrive(power);

        if(distance > 0) {
            while (robot.fl.getCurrentPosition() - flDist < distance &&
                    robot.fr.getCurrentPosition() - frDist< distance &&
                    robot.bl.getCurrentPosition() - blDist< distance &&
                    robot.br.getCurrentPosition() - brDist< distance) {
            }
        } else {
            while (robot.fl.getCurrentPosition() > distance &&
                    robot.fr.getCurrentPosition() > distance &&
                    robot.bl.getCurrentPosition() > distance &&
                    robot.br.getCurrentPosition() > distance) {
            }
        }

        stopDriving();
    }

    void moveForward(double power){
        robot.fl.setPower(power);
        robot.fr.setPower(power);
        robot.br.setPower(power);
        robot.bl.setPower(power);
    }
    void RotateDistance(double power, int distance) throws InterruptedException {
        {
            //reset encoders
            robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.fl.setTargetPosition(distance);
            robot.fr.setTargetPosition(-distance);
            robot.bl.setTargetPosition(distance);
            robot.br.setTargetPosition(-distance);

            robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rotateRight(power);

            while (robot.fl.isBusy() && robot.fr.isBusy() && robot.bl.isBusy() && robot.br.isBusy()) {
                //wait until robot stops
            }

            stopDriving();
        }
    }

     void gyroAlign0() throws InterruptedException{
         if(getHeading() > 1){
             while(getHeading() >= 0.5) {
                 robot.fl.setPower(0.15);
                 robot.bl.setPower(0.15);
             }
         }
         robot.fl.setPower(0);
         robot.bl.setPower(0);
     }

     void gyroAlign1() throws InterruptedException{
         if(getHeading() < 2){
             while(getHeading() <= 1.55) {
                 robot.fr.setPower(0.15);
                 robot.br.setPower(0.15);
             }
         }
         robot.fr.setPower(0);
         robot.br.setPower(0);
     }

    void gyroAlign90() throws InterruptedException{
        if(getHeading() < 88){
            while(getHeading() <= 88.5) {
                robot.fr.setPower(0.1);
                robot.br.setPower(0.1);
            }
        } else if(getHeading() > 92){
            while(getHeading()>91.5){
                robot.fr.setPower(-0.1);
                robot.br.setPower(-0.1);
            }
        }
        robot.fr.setPower(0);
        robot.br.setPower(0);
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle)  throws InterruptedException{

        int     newLeftTarget;
        int newLeftTargetB;
        int     newRightTarget;
        int newRightTargetB;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active

            robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * robot.ticksPerInch);
            //newLeftTarget = robot.bl.getCurrentPosition() + moveCounts;
            newLeftTarget = robot.fl.getCurrentPosition() + moveCounts;
            newRightTarget = robot.fr.getCurrentPosition() + moveCounts;
            newLeftTargetB = robot.bl.getCurrentPosition() + moveCounts;
            newRightTargetB = robot.br.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.bl.setTargetPosition(newLeftTarget);
            robot.fl.setTargetPosition(newLeftTarget);
            robot.br.setTargetPosition(newRightTarget);
            robot.fr.setTargetPosition(newRightTarget);

            robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion
            speed = Range.clip(Math.abs(speed), -1, 1.0);
            robot.bl.setPower(speed);
            robot.fl.setPower(speed);
            robot.fr.setPower(speed);
            robot.br.setPower(speed);

            //
        telVar = Boolean.toString(robot.fl.isBusy());
            while (
                    (robot.bl.isBusy() && robot.fr.isBusy() && robot.fl.isBusy() && robot.br.isBusy())) {
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, 0.008);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.bl.setPower(leftSpeed);
                robot.fl.setPower(leftSpeed);
                robot.br.setPower(rightSpeed);
                robot.fr.setPower(rightSpeed);


            }

            // Stop all motion;
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.fl.setPower(0);
            robot.bl.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public double getHeading() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Double.parseDouble(formatAngle(robot.angles.angleUnit, robot.angles.firstAngle));
    }
    public double normalize(double hi){
        if(hi < 0){
            hi = -hi;
            hi = 180-hi;
            hi += 180;
        }
        return hi;
    }

    public void gyroTurnRobotRightAbsolute(double angle, double power) {
        if(getHeading() <= 180 && getHeading() >= 180-angle-5){
            robot.br.setPower(-power);
            robot.fr.setPower(-power);
            robot.bl.setPower(power);
            robot.fl.setPower(power);
            while(normalize(getHeading()) > -(angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);

        } else {
            robot.br.setPower(-power);
            robot.fr.setPower(-power);
            robot.bl.setPower(power);
            robot.fl.setPower(power);
            while(getHeading() > -(angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);
        }
    }

    public void gyroTurnRobotLeftAbsolute(double angle, double power) {
        if(getHeading() <= 180 && getHeading() >= 180-angle-5){
            robot.br.setPower(power);
            robot.fr.setPower(power);
            robot.bl.setPower(-power);
            robot.fl.setPower(-power);
            while(normalize(getHeading()) > (angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);

        } else {
            robot.br.setPower(power);
            robot.fr.setPower(power);
            robot.bl.setPower(-power);
            robot.fl.setPower(-power);
            while(getHeading() > (angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);
        }
    }


    public void gyroTurnRobotRight(double angle, double power){
        double rightTurnHeading = getHeading();
        if(getHeading() <= 180 && getHeading() >= 180+angle+5){
            robot.bl.setPower(-power);
            robot.fl.setPower(-power);
            robot.br.setPower(power);
            robot.fr.setPower(power);
            while(normalize(getHeading()) > normalize(rightTurnHeading) - (angle-3)){
        }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);

        } else {
            robot.bl.setPower(power);
            robot.fl.setPower(power);
            robot.br.setPower(-power);
            robot.fr.setPower(-power);
            while(getHeading() > rightTurnHeading - (angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);
        }
    }
    public void gyroTurnRobotLeft(double angle, double power){
        double rightTurnHeading = getHeading();
        if(getHeading() >= -180 && getHeading() <= -180+(angle+5)){
            robot.br.setPower(power);
            robot.fr.setPower(power);
            robot.bl.setPower(-power);
            robot.fl.setPower(-power);
            while(normalize(getHeading()) < normalize(rightTurnHeading) + (angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);

        } else {
            robot.br.setPower(power);
            robot.fr.setPower(power);
            robot.bl.setPower(-power);
            robot.fl.setPower(-power);
            while(getHeading() < rightTurnHeading + (angle-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);
            robot.bl.setPower(0);
            robot.fl.setPower(0);
        }
    }

    void extendSlide(double power){
        robot.flipL.setPower(power);
        robot.flipR.setPower(-power);
    }*/


    //insert methods
}
