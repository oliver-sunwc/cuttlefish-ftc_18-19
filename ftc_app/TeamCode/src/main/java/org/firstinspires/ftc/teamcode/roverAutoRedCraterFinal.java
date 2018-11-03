package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static com.google.blocks.ftcrobotcontroller.util.HardwareType.BNO055IMU;
@Autonomous(name="autRedCraterFinal",group="rover")
public class roverAutoRedCraterFinal extends LinearOpMode {
    roverHMAP robot;
    roverAuto robotAuto;
    AnalogInput ods;
    String leftColor;
    String rightColor;
    int lwhite = 0;
    int lyellow = 0;
    int rwhite = 0;
    int ryellow = 0;
    private ExampleBlueVision blueVision;


    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        robot = new roverHMAP();
        robot.init(hardwareMap);
        robotAuto = new roverAuto(robot);

        robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        blueVision = new ExampleBlueVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        blueVision.setShowCountours(true);
        // start the vision system
        blueVision.enable();
        robot.gameMarker.setPosition(0.8);

        waitForStart();




        robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition());
        robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition());
        robot.flipL.setPower(0.1);
        robot.flipR.setPower(-0.1);

        robot.intakeDrop.setPosition(0.2);


        robot.hang.setTargetPosition(robot.hang.getCurrentPosition()-500);
        robot.hang.setPower(-0.6);
        Thread.sleep(750);
        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() + 1700);
        robot.hang.setPower(0.3);
        Thread.sleep(1000);
        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() + 1200);
        robot.hang.setPower(0.7);
        Thread.sleep(1000);



        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();

        //gyroDrive(0.1,18,0);
        //robotAuto.verticalDriveDistance(0.1,10);


        robot.intakeDrop.setPosition(1.0);
        robotAuto.verticalDriveDistance(0.2,1.7);
        sleep(100);
        robotAuto.gyroAlign0();
        Thread.sleep(100);
        robotAuto.moveForward(0.1);
        while(robot.dBack.getDistance(DistanceUnit.CM) < 5.9 ){

        }
        robotAuto.stopDriving();


        //robotAuto.verticalDriveDistance(0.1,3.5);
        Thread.sleep(100);
        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();

        robotAuto.gyroTurnRobotRight(19,0.3);
        while(robotAuto.getHeading()>-49){
            robot.fl.setPower(0.1);
            robot.bl.setPower(0.1);
            robot.fr.setPower(-0.1);
            robot.br.setPower(-0.1);
        }
        robotAuto.stopDriving();

        List<MatOfPoint> contours = blueVision.getContours();
        boolean left = false;
        boolean right = false;
        boolean center = false;

        contours = blueVision.getContours();
        telemetry.addData("gyro",getHeading());



        telemetry.addData("performing","left view");
        telemetry.update();
        int counter=0;
        telemetry.update();
        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();
        for(int i = 0;i< contours.size();i++){
            Rect boundRec = Imgproc.boundingRect(contours.get(i));
            if(boundRec.x + boundRec.width > 3*blueVision.hsv.width()/4) {
                counter++;
            }
            telemetry.addData("boundRec"+i,boundRec.x + boundRec.width);
        }
        if(counter > 0){
            left = true;
        } else {
            left = false;
        }
        telemetry.addData("left",left);
        telemetry.addData("width",blueVision.hsv.width());
        telemetry.addData("counter",counter);
        telemetry.update();

        //robotAuto.gyroTurnRobotRight(19,0.3);
        if(!left) {
            while (robotAuto.getHeading() > -95) {
                robot.fl.setPower(0.1);
                robot.bl.setPower(0.1);
                robot.fr.setPower(-0.1);
                robot.br.setPower(-0.1);
            }
            robotAuto.stopDriving();
            contours = blueVision.getContours();
            telemetry.addData("gyro", getHeading());
            telemetry.addData("performing", "center view");
            telemetry.update();
            Thread.sleep(100);
            for (int i = 0; i < contours.size(); i++) {
                Rect boundRec = Imgproc.boundingRect(contours.get(i));
                if (boundRec.x + boundRec.width > 4 * blueVision.hsv.width() / 5 && boundRec.y > blueVision.hsv.height()/5) {
                    counter++;
                }
                telemetry.addData("boundRec" + i, boundRec.x + boundRec.width);
            }
            telemetry.update();
            if (counter > 0) {
                center = true;
            } else {
                center = false;
            }
        }

        if(!left && !center) {
            right = true;
        }
        if(left) {
            telemetry.addData("left",left);
            telemetry.update();
            Thread.sleep(100);
            robot.fl.setPower(-0.1);
            robot.bl.setPower(-0.1);
            robot.fr.setPower(0.1);
            robot.br.setPower(0.1);
            while(robotAuto.getHeading() < -25){

            }
            robotAuto.stopDriving();

            robotAuto.verticalDriveDistance(0.1,1.5);

            robot.intakeDrop.setPosition(1.0);

            robot.fl.setPower(-0.1);
            robot.bl.setPower(-0.1);
            robot.fr.setPower(0.1);
            robot.br.setPower(0.1);
            while(robotAuto.getHeading() < 27){

            }
            robotAuto.stopDriving();

            robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() - 2200);
            robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() + 2200);
            robot.flipL.setPower(-0.5);
            robot.flipR.setPower(0.5);
            while(robot.flipL.isBusy() && robot.flipR.isBusy()) {
            }
            robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() + 2200);
            robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() - 2200);
            robot.flipL.setPower(0.5);
            robot.flipR.setPower(-0.5);

            while(robotAuto.getHeading() < 80) {
                robot.fr.setPower(0.17);
                robot.br.setPower(0.17);
            }
            robotAuto.stopDriving();

            robotAuto.gyroAlign90();

            robotAuto.verticalDriveDistance(0.2,26.5);

            /*robotAuto.verticalDrive(0.2);
            while(robot.dRight.getDistance(DistanceUnit.CM) > 48) {
                telemetry.addData("dR", robot.dRight.getDistance(DistanceUnit.CM));
                telemetry.update();
            }*/
            robotAuto.stopDriving();
            Thread.sleep(100);

            robot.fr.setPower(0.2);
            robot.br.setPower(0.2);
            while(robotAuto.getHeading() < 105){

            }
            robotAuto.stopDriving();
            Thread.sleep(100);
            robot.fr.setPower(0.07);
            robot.br.setPower(0.07);
            while(robotAuto.getHeading() < 127){

            }
            robotAuto.stopDriving();
            sleep(100);
            robotAuto.verticalDriveDistance(0.3,27);
            robot.gameMarker.setPosition(0);
            Thread.sleep(500);
            robotAuto.verticalDriveDistance(-0.5,-44);
        } else if(center){
            telemetry.addData("center",center);
            telemetry.update();

            robotAuto.gyroTurnRobotLeft(20,0.3);
            robot.fl.setPower(-0.1);
            robot.bl.setPower(-0.1);
            robot.fr.setPower(0.1);
            robot.br.setPower(0.1);
            while(robotAuto.getHeading() < -25){

            }
            robotAuto.stopDriving();

            robotAuto.verticalDriveDistance(0.1,3);
            robot.fl.setPower(-0.1);
            robot.bl.setPower(-0.1);
            robot.fr.setPower(0.1);
            robot.br.setPower(0.1);
            while(robotAuto.getHeading() > 1.5){

            }
            robotAuto.stopDriving();

            robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() - 1500);
            robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() + 1500);
            robot.flipL.setPower(-0.5);
            robot.flipR.setPower(0.5);
            while(robot.flipL.isBusy() && robot.flipR.isBusy()) {
            }
            robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() + 1500);
            robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() - 1500);
            robot.flipL.setPower(0.5);
            robot.flipR.setPower(-0.5);
            while(robotAuto.getHeading() < 80) {
                robot.fl.setPower(-0.15);
                robot.fl.setPower(-0.15);
                robot.fr.setPower(0.15);
                robot.br.setPower(0.15);
            }
            robotAuto.stopDriving();

            robotAuto.gyroAlign90();

            robotAuto.verticalDriveDistance(0.2,28);
            /*robotAuto.verticalDrive(0.2);
            while(robot.dRight.getDistance(DistanceUnit.CM) > 48) {
                telemetry.addData("dR", robot.dRight.getDistance(DistanceUnit.CM));
                telemetry.update();
            }*/
            robotAuto.stopDriving();
            Thread.sleep(100);

            robot.fr.setPower(0.2);
            robot.br.setPower(0.2);
            while(robotAuto.getHeading() < 105){

            }
            robotAuto.stopDriving();
            Thread.sleep(100);
            robot.fr.setPower(0.07);
            robot.br.setPower(0.07);
            while(robotAuto.getHeading() < 127){

            }
            robotAuto.stopDriving();
            sleep(100);
            robotAuto.verticalDriveDistance(0.3,27);
            robot.gameMarker.setPosition(0);
            Thread.sleep(2500);
            robotAuto.verticalDriveDistance(-0.5,-46);
        } else {
            telemetry.addData("right",right);
            telemetry.update();
            robotAuto.gyroTurnRobotLeft(20,0.3);
            robot.fl.setPower(-0.1);
            robot.bl.setPower(-0.1);
            robot.fr.setPower(0.1);
            robot.br.setPower(0.1);
            while(robotAuto.getHeading() < -50){

            }
            robotAuto.stopDriving();

            robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() - 2200);
            robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() + 2200);
            robot.flipL.setPower(-0.5);
            robot.flipR.setPower(0.5);
            while(robot.flipL.isBusy() && robot.flipR.isBusy()) {
            }
            robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() + 2200);
            robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() - 2200);
            robot.flipL.setPower(0.5);
            robot.flipR.setPower(-0.5);
            while(robotAuto.getHeading() < 80) {
                robot.fl.setPower(-0.15);
                robot.fl.setPower(-0.15);
                robot.fr.setPower(0.12);
                robot.br.setPower(0.12);
            }
            robotAuto.stopDriving();

            robotAuto.gyroAlign90();

            robotAuto.verticalDriveDistance(0.2,26.5);
            /*robotAuto.verticalDrive(0.2);
            while(robot.dRight.getDistance(DistanceUnit.CM) > 48) {
                telemetry.addData("dR", robot.dRight.getDistance(DistanceUnit.CM));
                telemetry.update();
            }*/
            robotAuto.stopDriving();
            Thread.sleep(100);

            robot.fr.setPower(0.2);
            robot.br.setPower(0.2);
            while(robotAuto.getHeading() < 105){

            }
            robotAuto.stopDriving();
            Thread.sleep(100);
            robot.fr.setPower(0.07);
            robot.br.setPower(0.07);
            while(robotAuto.getHeading() < 127){

            }
            robotAuto.stopDriving();
            sleep(100);
            robotAuto.verticalDriveDistance(0.3,27);
            robot.gameMarker.setPosition(0);
            Thread.sleep(2500);
            robotAuto.verticalDriveDistance(-0.5,-46);
        }
        sleep(100);



        /*while(Double.isNaN(robot.dRight.getDistance(DistanceUnit.CM))) {
            robotAuto.verticalDrive(0.4);
        }
        if(robot.dRight.getDistance(DistanceUnit.CM) < 15) {
            robotAuto.stopDriving();
        }

        sleep(150);

        while(robotAuto.getHeading() < 100) {
            robot.br.setPower(0.5);
            robot.fr.setPower(0.5);
        }*/



        //insert hit code

        /*robotAuto.verticalDriveDistance(-0.3,-25);
        robotAuto.gyroTurnRobotRight(0.2,60);
        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();
        Thread.sleep(2000);
        while(robotAuto.getHeading() > -10){
            robot.fl.setPower(0.1);
            robot.bl.setPower(0.1);
            robot.fr.setPower(-0.1);
            robot.br.setPower(-0.1);
        }
        robotAuto.stopDriving();
        //yeet the shit out of shit*/

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
            telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            telemetry.addData("Target",  "%7d:%7d",      newLeftTargetB,  newRightTargetB);
            telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            telemetry.addData("heading", getHeading());
            telemetry.update();

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
     */
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
}
