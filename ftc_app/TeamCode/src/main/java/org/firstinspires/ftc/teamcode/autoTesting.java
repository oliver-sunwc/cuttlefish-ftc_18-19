package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

import java.util.List;
import java.util.Locale;

@Autonomous(name="autoTesting")
public class autoTesting extends LinearOpMode {
    roverHMAP robot;
    roverAuto robotAuto;
    private VisionThing vision;


    public void runOpMode() throws InterruptedException {
        robot = new roverHMAP();
        robot.init(hardwareMap,true);
        robotAuto = new roverAuto(robot);


        robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        double p = 12;
        double i = 0;
        double d = 0;
        boolean testa = false;
        boolean testb = false;
        boolean testc = false;
        boolean testd = false;
        boolean teste = false;
        boolean testf = false;
        while(true){
            if(gamepad1.dpad_up && !testa){
                p += 0.1;
            }
            testa = gamepad1.dpad_up;

            if(gamepad1.dpad_down && !testb){
                p -= 0.1;
            }
            testb = gamepad1.dpad_down;



            if(gamepad1.y && !testc){
                d += 0.1;
            }
            testc = gamepad1.y;

            if(gamepad1.a && !testd){
                d -= 0.1;
            }
            testd = gamepad1.a;



            if(gamepad1.b && !teste){
                i += 0.1;
            }
            teste = gamepad1.b;

            if(gamepad1.x && !testf){
                i -= 0.1;
            }
            testf = gamepad1.x;



            if(isStarted()){
                break;
            }

            telemetry.addData("p",p);
            telemetry.addData("i",i);
            telemetry.addData("d",d);
            telemetry.update();
        }
        waitForStart();
        telemetry.addData("fl",robot.fl.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
        telemetry.update();


        PIDCoefficients hi = new PIDCoefficients(p,i,d);

        robot.fl.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, hi);
        robot.fr.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, hi);
        robot.bl.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, hi);
        robot.br.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, hi);

        robotAuto.stopAndReset();

        robotAuto.runToPosition();






        robot.fl.setTargetPosition(3000);
        robot.fr.setTargetPosition(95);
        robot.bl.setTargetPosition(3000);
        robot.br.setTargetPosition(95);

        robot.fl.setPower(0.95);
        robot.bl.setPower(0.95);
        robot.fr.setPower(0.03);
        robot.br.setPower(0.03);

        robot.fl.setTargetPositionTolerance(10);
        robot.bl.setTargetPositionTolerance(10);
        robot.fr.setTargetPositionTolerance(10);
        robot.br.setTargetPositionTolerance(10);

//12 0.2 1
        //12 2 2
        //12 0.5 1.5
        while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){


        }
        telemetry.addData("fl",robot.fl.getCurrentPosition());
        telemetry.addData("bl",robot.bl.getCurrentPosition());
        telemetry.addData("br",robot.br.getCurrentPosition());
        telemetry.addData("fr",robot.fr.getCurrentPosition());
        telemetry.addData("fl",robot.fl.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
        telemetry.update();

        robotAuto.verticalDrive(0);
        telemetry.addData("done","done");
        telemetry.update();

        robotAuto.stopAndReset();
        Thread.sleep(10);
        robotAuto.runToPosition();
        Thread.sleep(10);

        telemetry.addData("fl",robot.fl.getCurrentPosition());
        telemetry.addData("bl",robot.bl.getCurrentPosition());
        telemetry.addData("br",robot.br.getCurrentPosition());
        telemetry.addData("fr",robot.fr.getCurrentPosition());
        telemetry.addData("fl",robot.fl.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
        telemetry.update();

        robot.fl.setTargetPosition(2700);
        robot.bl.setTargetPosition(2700);
        robot.fr.setTargetPosition(1333);
        robot.br.setTargetPosition(1333);

        robot.fl.setPower(0.9);
        robot.bl.setPower(0.9);
        robot.br.setPower(0.5);
        robot.fr.setPower(0.5);

        while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){
            if(robot.fl.getCurrentPosition() > 150){

            }

        }

        robotAuto.verticalDrive(0);
        Thread.sleep(50);

        robotAuto.runUsing();

        robot.fl.setPower(-0.5);
        robot.bl.setPower(-0.5);
        robot.fr.setPower(0.1);
        robot.br.setPower(0.1);
        while(robotAuto.getHeading() > 98){

        }

        robotAuto.stopDriving();
        Thread.sleep(50);

        robotAuto.stopAndReset();
        Thread.sleep(10);
        robotAuto.runToPosition();
        Thread.sleep(10);

        robot.fl.setTargetPosition(-2200);
        robot.bl.setTargetPosition(-2200);
        robot.fr.setTargetPosition(-2200);
        robot.br.setTargetPosition(-2200);

        robot.fl.setPower(-0.3);
        robot.bl.setPower(-0.3);
        robot.br.setPower(-0.3);
        robot.fr.setPower(-0.3);

        while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){
            if(robot.fl.getCurrentPosition() < -100){
                robot.fl.setPower(-0.9);
                robot.bl.setPower(-0.9);
                robot.fr.setPower(-0.9);
                robot.br.setPower(-0.9);
            }

        }

        robotAuto.stopDriving();
        Thread.sleep(50);
    }

    void checkStop(){
        if(isStopRequested()){
            vision.disable();
        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

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

}

