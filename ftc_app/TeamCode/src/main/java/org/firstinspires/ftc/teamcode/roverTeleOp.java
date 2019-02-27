package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.MediaPlayer;
import android.media.ToneGenerator;

import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.*;
import java.util.Locale;

/**
 * Manual with Arcade Drive
 */
@TeleOp(name = "TankFinal", group = "Rover")
public class roverTeleOp extends OpMode {

    roverHMAP robot = new roverHMAP();
    double rx,ry,lx;
    double hang;
    double hangPow = 1;

    boolean fliptog = false;
    boolean f;
    boolean rotatetog = false;
    boolean r;
    boolean ninja = false;
    boolean n;
    int intakeTog = 0;
    boolean i;
    int flipAng = 325;
    boolean inNinja = false;
    boolean iN;
    boolean midTog = false;
    boolean m;
    boolean pressed = false;
    boolean inFlipTrigger1 = false;
    boolean inFlipTrigger2 = false;
    boolean re;

    boolean hangModeStall = false;
    boolean h = false;

    boolean flapUp = true;

    ToneGenerator tone = new ToneGenerator(AudioManager.STREAM_MUSIC,100);
    MediaPlayer mp = new MediaPlayer();

    boolean songStarted = false;
    boolean s = false;

    int initialPosition;
    ElapsedTime stallTime = new ElapsedTime();
    ElapsedTime flipTimer = new ElapsedTime();
    ElapsedTime inFlipTimer = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap,false);
        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rotateArm.setPosition(0.5);

        mp = MediaPlayer.create(this.hardwareMap.appContext,R.raw.thatway);
        mp.seekTo(0);//initialPosition = robot.hang.getCurrentPosition();
    }

    @Override
    public void start(){
        //robot.hang.setTargetPosition(initialPosition - 1000);
        robot.rotateArm.setPosition(0.5);

    }

    @Override
    public void loop(){

        if(!s && gamepad2.left_stick_button){
            if(songStarted){
                songStarted = false;
                mp.pause();
            } else {
                songStarted = true;
                mp.start();
            }
        }

        s= gamepad2.left_stick_button;

        telemetry.addData("hang",robot.hang.getCurrentPosition());
        //toggle button for intake speed
        if(gamepad2.right_bumper && !iN) {
            if(!inNinja) {
                inNinja = true;
            } else {
                inNinja = false;
            }
        }


        iN = gamepad2.right_bumper;

        if(gamepad2.left_trigger > 0.5){
            flapUp = false;
        } else {
            flapUp=true;
        }

        if(flapUp){
            robot.flap.setPosition(0.5);
        } else {
            robot.flap.setPosition(0.7);
        }


        if(!re && gamepad1.b) {
            robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        re = gamepad1.b;

        /*if(gamepad1.y) {
            robot.inFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.inFlip.setPower(0.4);
        } else {
            robot.inFlip.setPower(0);
        }

        /*if(gamepad1.x) {
            robot.inFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.inFlip.setPower(-0.65);
        } else {
            robot.inFlip.setPower(0);
        }*/

        //intake flip toggle position
        if((gamepad2.a || gamepad1.left_bumper) && !i) {
            if(intakeTog == 0) {
                inFlipTimer.reset();
                intakeTog = 1;
                robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.inFlip.setPower(0.35);
                robot.inFlip.setTargetPosition(flipAng);
                inFlipTrigger1 = true;
            } else if(intakeTog == 1) {
                inFlipTimer.reset();
                intakeTog = 0;
                robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.inFlip.setPower(-0.55);
                robot.inFlip.setTargetPosition(-flipAng);
                inFlipTrigger2 = true;
            }
        }


        if(inFlipTrigger1 && inFlipTimer.seconds()>0.3){
            inFlipTrigger1 = false;
            robot.inFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.inFlip.setPower(0);
        }

        /*if(inFlipTrigger2 && inFlipTimer.seconds() < 0.8){
            robot.inFlip.setPower(-0.45*((robot.inFlip.getCurrentPosition() + flipAng)/(flipAng)) -0.02);
        }*/

        if(inFlipTrigger2 && inFlipTimer.seconds() > 0.8){
            inFlipTrigger2 = false;
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition()-1);
            robot.inFlip.setPower(-0.07);
        }

        if(inFlipTimer.seconds()>1.2){
            inFlipTrigger1=false;
            inFlipTrigger2 = false;
        }

        i = (gamepad2.a || gamepad1.left_bumper);

        /*if(!m && gamepad2.b) {
            if(!midTog) {
                midTog = true;
                if(intakeTog == 0) {
                    robot.inFlip.setPower(0.9);
                    robot.inFlip.setTargetPosition(flipAng/2);
                } else {
                    robot.inFlip.setPower(-1);
                    robot.inFlip.setTargetPosition(flipAng/2);
                }
                intakeTog = 0;
            } else {
                midTog = false;
                intakeTog = 0;
                robot.inFlip.setPower(-1);
                robot.inFlip.setTargetPosition(0);
            }
        }

        m = gamepad2.b;*/

        // do based on intake position

        if(!h && gamepad2.b){
            hangModeStall = !hangModeStall;
        }

        h = gamepad2.b;

        if(hangModeStall){
            hangPow = 0.1;
        } else {
            hangPow = 1;
        }


        //flip toggle position
        if(!f && gamepad2.x) {
            pressed = true;
            if(!fliptog) {
                robot.flipLArm.setPosition(1);
                robot.flipRArm.setPosition(0);
                flipTimer.reset();
                fliptog = true;
            } else {
                robot.flipLArm.setPosition(0);
                robot.flipRArm.setPosition(1);
                flipTimer.reset();
                fliptog = false;
            }
        }
        f = gamepad2.x;

        if(fliptog && flipTimer.milliseconds() >= 300 && pressed) {
            robot.rotateArm.setPosition(0.5);
            pressed = false;
        } else if(!fliptog && flipTimer.milliseconds() >= 300 && pressed) {
            robot.rotateArm.setPosition(0);
            pressed = false;
        }

        if(!r && gamepad2.y) {

            if(!rotatetog) {
                rotatetog = true;
                robot.flipLArm.setPosition(0.1);
                robot.flipRArm.setPosition(0.9);
                robot.rotateArm.setPosition(0.5);
            } else {
                robot.flipLArm.setPosition(1);
                robot.flipRArm.setPosition(0);
                robot.rotateArm.setPosition(0.5);
                rotatetog = false;
            }
        }

        r = gamepad2.y;



        // ninja thing
        ninja = gamepad1.right_bumper;

        // drive the robot code
        rx = Math.pow(gamepad1.right_stick_x, 1);
        ry = -Math.pow(gamepad1.right_stick_y, 1);
        lx = -Math.pow(gamepad1.left_stick_x, 1);


        if(!gamepad1.dpad_left && !gamepad2.dpad_right) {
            if (!ninja) {
                mecanumDrive(rx, ry, lx, 0);
            } else {
                mecanumDrive(rx / 3, ry / 3, lx / 3, 0);
            }
        }

        if(gamepad1.dpad_right){
            mecanumDrive(0,0,-0.1,0);
        } else if(gamepad1.dpad_left){
            mecanumDrive(0,0,0.1,0);
        }
        // hang code
        if(gamepad2.dpad_down){
            robot.hang.setPower(hangPow);

        } else if(gamepad2.dpad_up){
            robot.hang.setPower(-hangPow);
        } else{
            robot.hang.setPower(0);
        }


        // spine code
        robot.spine.setPower(Range.clip(gamepad2.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger,-1,1));

        //run intake code
        if(inNinja) {
            robot.intake.setPower(gamepad2.right_stick_y/3.2);
        } else {
            robot.intake.setPower(gamepad2.right_stick_y/1.6);
        }

        if(gamepad2.left_trigger != 0.1 && gamepad2.right_trigger > 0.1) {
            hangStall();
        }

        //telemetry.addData("gyro",getHeading());
        telemetry.addData("lx", lx);
        telemetry.addData("ry", ry);
        telemetry.addData("rx", rx);
        telemetry.addData("thing",robot.inFlip.getCurrentPosition());
        telemetry.addData("target",robot.inFlip.getTargetPosition());
        telemetry.addData("timer",inFlipTimer.seconds());
        telemetry.addData("mode",robot.inFlip.getMode());
        telemetry.update();

    }

    void mecanumDrive(double lx,double ly,double rx,double k){
        robot.fl.setPower(Range.clip((1+k)*(ly + rx - lx),-1,1));
        robot.bl.setPower(Range.clip((1-k)*(ly + rx + lx),-1,1));
        robot.fr.setPower(Range.clip((1+k)*(ly - rx + lx),-1,1));
        robot.br.setPower(Range.clip((1-k)*(ly - rx - lx),-1,1));
    }

    void hangStall() {
        stallTime.reset();
        if(stallTime.seconds() < 3) {
            robot.hang.setPower(0.4);
        } else {
            robot.hang.setPower(0);
        }
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
