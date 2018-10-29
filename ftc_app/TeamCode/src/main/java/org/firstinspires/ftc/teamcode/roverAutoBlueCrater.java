package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

import static com.google.blocks.ftcrobotcontroller.util.HardwareType.BNO055IMU;
@Autonomous(name="autBlueCrater",group="rover")
public class roverAutoBlueCrater extends LinearOpMode {
    roverHMAP robot;
    roverAuto robotAuto;
    AnalogInput ods;
    String leftColor;
    String rightColor;
    int lwhite = 0;
    int lyellow = 0;
    int rwhite = 0;
    int ryellow = 0;

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

        waitForStart();



        robot.intakeServo.setPosition(0.6);

        robot.hang.setTargetPosition(robot.hang.getCurrentPosition()-500);
        robot.hang.setPower(-0.6);
        Thread.sleep(750);
        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() + 1700);
        robot.hang.setPower(0.3);
        Thread.sleep(1000);
        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() + 950);
        robot.hang.setPower(0.5);
        Thread.sleep(2000);

        //robot.hang.setPower(-1);
        //Thread.sleep(4000);
        //robot.hang.setPower(0);*/
        robot.MArmL.setPosition(0.76);
        robot.MArmR.setPosition(0.10);

        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();

        //gyroDrive(0.1,18,0);
        //robotAuto.verticalDriveDistance(0.1,10);
        Thread.sleep(500);

        //run till close
        //robotAuto.gyroAlign1();
        //Thread.sleep(1000);
        robotAuto.verticalDriveDistance(0.2,11.2);
        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();

        //robotAuto.gyroAlign0();



        robotAuto.verticalDrive(0.15);
        do {
            if (!Double.isNaN(robot.dMArmL.getDistance(DistanceUnit.CM)) || !Double.isNaN(robot.dMArmR.getDistance(DistanceUnit.CM))) {
                robotAuto.stopDriving();
                break;
            }

            telemetry.addData("dl",robot.dMArmL.getDistance(DistanceUnit.CM));
            telemetry.addData("dr",robot.dMArmR.getDistance(DistanceUnit.CM));
            telemetry.addData("heading", robotAuto.getHeading());
             telemetry.addData("stage",11);
            telemetry.update();
        } while(true);


        for(int i=0;i<1;i++) {
            robotAuto.verticalDrive(0.12);
            do {
                if (robot.dMArmL.getDistance(DistanceUnit.CM) < 45 || robot.dMArmR.getDistance(DistanceUnit.CM) < 45) {
                    robotAuto.stopDriving();
                    break;
                }

                telemetry.addData("dl", robot.dMArmL.getDistance(DistanceUnit.CM));
                telemetry.addData("dr", robot.dMArmR.getDistance(DistanceUnit.CM));
                telemetry.addData("heading", robotAuto.getHeading());
                telemetry.addData("stage", 10);
                telemetry.update();
            } while (true);

            while (Double.isNaN(robot.dMArmL.getDistance(DistanceUnit.CM))) {
                robot.fl.setPower(0.06);
                robot.bl.setPower(0.06);
                //robot.fr.setPower(-0.02);
                //robot.br.setPower(-0.02);
                telemetry.addData("dl", robot.dMArmL.getDistance(DistanceUnit.CM));
                telemetry.addData("dr", robot.dMArmR.getDistance(DistanceUnit.CM));
                telemetry.addData("heading", robotAuto.getHeading());
                telemetry.addData("stage", 12);
                telemetry.update();
            }
            robotAuto.stopDriving();
            while (Double.isNaN(robot.dMArmR.getDistance(DistanceUnit.CM))) {
                robot.fr.setPower(0.06);
                robot.br.setPower(0.06);
                //robot.fl.setPower(-0.02);
                //robot.br.setPower(-0.02);
                telemetry.addData("dl", robot.dMArmL.getDistance(DistanceUnit.CM));
                telemetry.addData("dr", robot.dMArmR.getDistance(DistanceUnit.CM));
                telemetry.addData("heading", robotAuto.getHeading());
                telemetry.addData("stage", 13);
                telemetry.update();
            }
            Thread.sleep(3000);
            robotAuto.stopDriving();
            while (robot.dMArmL.getDistance(DistanceUnit.CM) > 8) {
                robot.fl.setPower(0.04);
                robot.bl.setPower(0.04);
                robot.fr.setPower(-0.02);
                robot.br.setPower(-0.02);
                telemetry.addData("dl", robot.dMArmL.getDistance(DistanceUnit.CM));
                telemetry.addData("dr", robot.dMArmR.getDistance(DistanceUnit.CM));
                telemetry.addData("heading", robotAuto.getHeading());
                telemetry.addData("stage", 14);
                telemetry.update();
            }
            robotAuto.stopDriving();


            while (robot.dMArmR.getDistance(DistanceUnit.CM) > 8) {
                robot.fr.setPower(0.04);
                robot.br.setPower(0.04);
                robot.fl.setPower(-0.02);
                robot.bl.setPower(-0.02);
                telemetry.addData("dl", robot.dMArmL.getDistance(DistanceUnit.CM));
                telemetry.addData("dr", robot.dMArmR.getDistance(DistanceUnit.CM));
                telemetry.addData("heading", robotAuto.getHeading());
                telemetry.addData("stage", 15);
                telemetry.update();
            }
            robotAuto.stopDriving();
        }
        sleep(500);
        telemetry.addData("stage one", "complete");
        telemetry.addData("heading", robotAuto.getHeading());
        telemetry.update();


        // sense the color
        timer.reset();
        while(timer.seconds() < 1) {
            if (robot.cMArmL.red() < 1.3 * robot.cMArmL.blue()) {
                leftColor = "white";
                lwhite++;
            } else {
                leftColor = "yellow";
                lyellow++;
            }
            telemetry.addData("left color", leftColor);
            if (robot.cMArmR.red() < 1.3 * robot.cMArmR.blue()) {
                rightColor = "white";
                lwhite++;
            } else {
                rightColor = "yellow";
                lyellow++;
            }
            telemetry.addData("right color", rightColor);
            telemetry.addData("heading", robotAuto.getHeading());
            telemetry.update();
            Thread.sleep(50);
        }

        boolean middle;
        boolean left = false;
        //color sense program
        if(leftColor.equals("white") && rightColor.equals("white")) {
            robot.MArmL.setPosition(0.12);
            robot.MArmR.setPosition(0.75);
            middle = true;
        } else if(rightColor.equals("white")) {
            robot.MArmR.setPosition(0.75);
            left = true;
            middle = false;
        } else {
            robot.MArmL.setPosition(0.12);
            middle = false;
            left = false;
        }

        if(!middle) {

            robotAuto.verticalDriveDistance(0.25, 7);
            Thread.sleep(500);
            robot.MArmR.setPosition(0.75);
            robot.MArmL.setPosition(0.12);
            //robotAuto.verticalDriveDistance(-0.25, -12.2);
        } else {
            robotAuto.verticalDriveDistance( -0.3,-5.0);
            robot.intakeServo.setPosition(0.15);
            Thread.sleep(1000);
            robot.intake.setPower(1);
            Thread.sleep(3000);
            robot.intake.setPower(0);
            /*robotAuto.verticalDriveDistance(-0.25,-0.5);
            robot.intakeServo.setPosition(0.6); */
        }
        robot.intakeServo.setPosition(0.15);
        /*robotAuto.gyroTurnRobotLeft(90,0.1);
        robotAuto.verticalDriveDistance(0.4,35.5);
        double initHeading = getHeading();
        telemetry.addData("init",initHeading);
        telemetry.addData("init",initHeading-42);
        telemetry.update();
        while(robotAuto.getHeading()<135){
            telemetry.addData("init",initHeading);
            telemetry.addData("init",initHeading-42);
            telemetry.update();
            robot.fr.setPower(0.1);
            robot.br.setPower(0.1);
        }
        robotAuto.verticalDriveDistance(0.7,56);
        Thread.sleep(1000);
        robotAuto.verticalDriveDistance(-0.7,-56);

        robotAuto.gyroDrive(0.4, 69, -45);

        double rightTurnHeading = robotAuto.getHeading();
        if(robotAuto.getHeading() >= -180 && robotAuto.getHeading() <= -180+(90+5)){
            robot.br.setPower(0.2);
            robot.fr.setPower(0.2);
            while(robotAuto.normalize(robotAuto.getHeading()) < robotAuto.normalize(rightTurnHeading) + (90-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);

        } else {
            robot.br.setPower(0.2);
            robot.fr.setPower(0.2);
            while(robotAuto.getHeading() < rightTurnHeading + (90-3)){
            }
            robot.fr.setPower(0);
            robot.fl.setPower(0);
        }

        robotAuto.verticalDriveDistance(1, 69);
        //yeet the marker
        robotAuto.verticalDriveDistance(-1, -69);*/
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
