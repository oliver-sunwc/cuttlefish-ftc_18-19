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


        double p = 7.5;
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
                d += 0.01;
            }
            testc = gamepad1.y;

            if(gamepad1.a && !testd){
                d -= 0.01;
            }
            testd = gamepad1.y;



            if(gamepad1.b && !teste){
                i += 0.01;
            }
            teste = gamepad1.b;

            if(gamepad1.x && !testf){
                i -= 0.01;
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

        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Thread.sleep(250);

        robot.fl.setTargetPosition(1400);
        robot.fr.setTargetPosition(1400);
        robot.bl.setTargetPosition(1400);
        robot.br.setTargetPosition(1400);
        robotAuto.verticalDrive(0.9);



        while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){


        }

        robotAuto.verticalDrive(0);
        telemetry.addData("done","done");
        telemetry.update();
        Thread.sleep(1000);


        telemetry.addData("target",1400);
        telemetry.addData("fl",robot.fl.getCurrentPosition());
        telemetry.addData("bl",robot.bl.getCurrentPosition());
        telemetry.addData("br",robot.br.getCurrentPosition());
        telemetry.addData("fr",robot.fr.getCurrentPosition());
        telemetry.addData("fl",robot.fl.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
        telemetry.update();

        Thread.sleep(20000);


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

