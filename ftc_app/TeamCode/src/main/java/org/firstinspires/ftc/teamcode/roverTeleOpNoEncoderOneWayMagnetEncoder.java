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
@TeleOp(name = "TankFinalRealWereGoingToWorldsTankFinalRealWereGoingToWorldsTankFinalRealWereGoingToWorldsTankFinalRealWereGoingToWorldsTankFinalRealWereGoingToWorldsTankFinalRealWereGoingToWorldsTankFinalRealWereGoingToWorldsTankFinalRealWereGoingToWorldsTankFinalRealWereGoingToWorldsTankFinalRealWereGoingToWorldsTankFinalRealWereGoingToWorldsTankFinalRealWereGoingToWorlds", group = "Rover")
public class roverTeleOpNoEncoderOneWayMagnetEncoder extends OpMode {

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
    int flipAng = 280;
    boolean inNinja = false;
    boolean iN;
    boolean midTog = false;
    boolean m;
    boolean pressed = false;
    boolean inFlipTrigger1 = false;
    boolean inFlipTrigger2 = false;
    boolean re;

    String song = "TTFAF";

    boolean u = false;

    ElapsedTime timerBitch = new ElapsedTime();

    boolean hangModeStall = false;
    boolean h = false;

    boolean flapUp = true;

    boolean w = false;

    int counter = 0;

    int pos = 0; // 0 is down 1 is x 2 is y;

    MediaPlayer mp6 = new MediaPlayer();
    MediaPlayer mp5 = new MediaPlayer();
    MediaPlayer mp4 = new MediaPlayer();
    MediaPlayer mp3 = new MediaPlayer();
    MediaPlayer mp1 = new MediaPlayer();
    MediaPlayer mp2 = new MediaPlayer();

    boolean triggerModeSwitch = false;
    boolean triggerModeSwitch2 = false;

    boolean songStarted = false;
    boolean s = false;
    boolean ni = false;

    int initialPosition;
    ElapsedTime stallTime = new ElapsedTime();
    ElapsedTime flipTimer = new ElapsedTime();
    ElapsedTime inFlipTimer = new ElapsedTime();
    ElapsedTime failsafe = new ElapsedTime();
    boolean fail = false;
    boolean fail2 = false;

    @Override
    public void init() {
        robot.init(hardwareMap,false);
        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.inFlip.setTargetPosition(0);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.rotateArm.setPosition(0.5);

        mp6 = MediaPlayer.create(this.hardwareMap.appContext,R.raw.bakemonogatari);
        mp5 = MediaPlayer.create(this.hardwareMap.appContext,R.raw.newthatway);
        mp4 = MediaPlayer.create(this.hardwareMap.appContext,R.raw.ttfaf);
        mp3 = MediaPlayer.create(this.hardwareMap.appContext,R.raw.ocean_man);
        mp1 = MediaPlayer.create(this.hardwareMap.appContext,R.raw.gold);
        mp2 = MediaPlayer.create(this.hardwareMap.appContext,R.raw.silver);

        mp3.seekTo(0);
        mp4.seekTo(0);
        mp5.seekTo(0);
        mp6.seekTo(0);
    }

    @Override
    public void start(){
        //robot.hang.setTargetPosition(initialPosition - 1000);
        robot.rotateArm.setPosition(0.75);
        failsafe.startTime();

    }

    @Override
    public void loop(){

        if(!s && gamepad2.left_stick_button){
            if(songStarted){
                if (song == "TTFAF") {
                    mp4.stop();
                } else if (song == "iwity") {
                    mp5.stop();
                } else if (song == "ocean man") {
                    mp3.stop();
                } else if (song == "SE NO") {
                    mp6.stop();
                }
                songStarted = false;
            } else {
                if (song == "TTFAF") {
                    mp4.start();
                } else if (song == "iwity") {
                    mp5.start();
                } else if (song == "ocean man") {
                    mp3.start();
                } else if (song == "SE NO") {
                    mp6.start();
                }
                songStarted = true;
            }
        }

        s = gamepad2.left_stick_button;

        if(!ni && gamepad2.right_stick_button) {
            if (song == "TTFAF") {
                mp4.stop();
                mp4.seekTo(0);
                song = "iwity";
                mp5.start();
            } else if (song == "iwity") {
                mp5.stop();
                mp5.seekTo(0);
                song = "ocean man";
                mp3.start();
            } else if (song == "ocean man") {
                mp3.stop();
                mp3.seekTo(0);
                song = "SE NO";
                mp6.start();
            } else if (song == "SE NO") {
                mp6.stop();
                mp6.seekTo(0);
                song = "TTFAF";
                mp4.start();
            }
            songStarted = true;
        }

        ni = gamepad2.right_stick_button;

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

        /*if(flapUp){
            robot.flap.setPosition(0.5);
        } else {
            robot.flap.setPosition(0.7);
        }*/


        /*if(!re && gamepad1.b) {
            robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }*/

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



        if(inFlipTrigger1 && robot.inFlip.getCurrentPosition() > flipAng){
            inFlipTrigger1 = false;
            //robot.inFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.inFlip.setPower(0);
        }

        /*if(inFlipTrigger2 && inFlipTimer.seconds() < 0.8){
            robot.inFlip.setPower(-0.45*((robot.inFlip.getCurrentPosition() + flipAng)/(flipAng)) -0.02);
        }*/

        if(triggerModeSwitch && inFlipTimer.seconds()>0.3){
            triggerModeSwitch = false;
            robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            triggerModeSwitch2 = true;
        }

        if(triggerModeSwitch2 && inFlipTimer.seconds() > 0.5){
            triggerModeSwitch2 = false;
            robot.inFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



        if(inFlipTrigger2 && robot.inFlip.getCurrentPosition() < 30){
            inFlipTrigger2 = false;
            //robot.inFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.inFlip.setPower(0);

            triggerModeSwitch = true;
            inFlipTimer.reset();
        }





        if((gamepad2.a || gamepad1.b) && !i){
            inFlipTimer.reset();
            intakeTog = 0;

            robot.inFlip.setPower(-0.85);

            inFlipTrigger2 = true;
        }
        i = (gamepad2.a || gamepad1.b);



        if(gamepad1.left_bumper && !w){
            inFlipTimer.reset();
            intakeTog = 1;

            robot.inFlip.setPower(0.65);

            inFlipTrigger1 = true;
        }

        w = gamepad1.left_bumper;


        if(gamepad1.y && !u){
            intakeTog = 0;
            failsafe.reset();
            robot.inFlip.setPower(-0.85);
            fail = true;
        }

        u = gamepad1.y;

        if(failsafe.seconds() > 0.6 && fail){
            fail = false;
            robot.inFlip.setPower(0);
            robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fail2 = true;
        }

        if(fail2 && failsafe.seconds() > 0.8){
            robot.inFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }




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
            if(pos == 1) {
                robot.flipLArm.setPosition(1);
                robot.flipRArm.setPosition(0);
                flipTimer.reset();
                pos = 0;
            } else if(pos == 0) {
                mp1.start();
                robot.flipLArm.setPosition(0);
                robot.flipRArm.setPosition(1);
                flipTimer.reset();
                pos = 1;
            } else if(pos == 2){
                robot.rotateArm.setPosition(0.25);
                robot.flipLArm.setPosition(0);
                robot.flipRArm.setPosition(1);
                pos = 1;
            }
        }
        f = gamepad2.x;

        if(pos == 0 && flipTimer.milliseconds() >= 300 && pressed) {
            robot.rotateArm.setPosition(0.75);
            pressed = false;
        } else if(pos == 1 && flipTimer.milliseconds() >= 300 && pressed) {
            robot.rotateArm.setPosition(0.25);
            pressed = false;
        }

        if(!r && gamepad2.y) {

            if(pos == 0) {
                rotatetog = true;
                robot.flipLArm.setPosition(0.1);
                robot.flipRArm.setPosition(0.9);
                robot.rotateArm.setPosition(0.75);
                mp2.start();
                pos = 2;
                /*robot.flap.setPosition(0.7);
                timerBitch.reset();*/

            } else if(pos==2){

                robot.flipLArm.setPosition(1);
                robot.flipRArm.setPosition(0);
                robot.rotateArm.setPosition(0.75);

                pos=0;

            } else if(pos==1){
                robot.flipLArm.setPosition(0.1);
                robot.flipRArm.setPosition(0.9);
                robot.rotateArm.setPosition(0.75);
                pos =2;
            }
        }

        r = gamepad2.y;




        /*if(timerBitch.seconds() > 0.9){
            if (flapUp) {
                robot.flap.setPosition(0.5);
            } else {
                robot.flap.setPosition(0.7);
            }
        } else {
            robot.flap.setPosition(0.7);
        }*/
        if (flapUp) {
            robot.flap.setPosition(0.5);
        } else {
            robot.flap.setPosition(0.7);
        }



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
        double intakePow = gamepad2.right_stick_y;


        if(inNinja) {
            intakePow/=2.4;
        } else {
            intakePow/=1;
        }

        if(intakePow > 0){
            intakePow/= 1.6;
        }

        if(intakeTog == 0){

        }

        intakePow = intakePow/0.9;
        robot.intake.setPower(intakePow);

        if(gamepad2.left_trigger != 0.1 && gamepad2.right_trigger > 0.1) {
            hangStall();
        }

        //telemetry.addData("gyro",getHeading());
        telemetry.addData("pos",pos);
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
