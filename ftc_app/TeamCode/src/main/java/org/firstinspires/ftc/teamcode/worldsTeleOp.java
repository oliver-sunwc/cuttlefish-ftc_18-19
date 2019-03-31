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
@TeleOp(name = "worldsTeleOp", group = "Rover")
public class worldsTeleOp extends OpMode {

    roverHMAP robot = new roverHMAP();
    double rx,ry,lx;
    double hangPow = 1;

    boolean f;

    boolean ninja = false;

    boolean inNinja = false;
    boolean iN;


    boolean hangModeStall = false;
    boolean h = false;


    boolean flipDown = true;

    boolean inFlipUp = false;
    boolean i;

    boolean trapDoorUp = true;
    boolean t;
    int pos = 0; // 0 is down 1 is x 2 is y;

    ElapsedTime timer1 = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    ElapsedTime timer3 = new ElapsedTime();
    boolean dumpTrigger = false;
    boolean dumpTrigger2 = false;
    boolean downStall = false;

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

        //region hang stall and power code
        telemetry.addData("hang",robot.hang.getCurrentPosition());

        if(!h && gamepad2.b){
            hangModeStall = !hangModeStall;
        }

        h = gamepad2.b;

        if(hangModeStall){
            hangPow = 0.1;
        } else {
            hangPow = 1;
        }

        if(gamepad2.dpad_down){
            robot.hang.setPower(hangPow);

        } else if(gamepad2.dpad_up){
            robot.hang.setPower(-hangPow);
        } else{
            robot.hang.setPower(0);
        }

        //endregion

        //region intake flip toggle
        if(!i && gamepad2.a){
            inFlipUp = !inFlipUp;
        }

        i = gamepad2.a;

        if(inFlipUp){
            robot.inFlip.setPosition(0.7);
        } else {
            robot.inFlip.setPosition(0);
        }
        //endregion

        // region trapDoor toggle
        if(!t && gamepad2.y){
            trapDoorUp = !trapDoorUp;
        }

        t = gamepad2.y;

        if(trapDoorUp){
            robot.trapDoor.setPosition(0.75);
        } else {
            robot.trapDoor.setPosition(0.1);
        }
        //endregion

        //region dump toggle
        if(!f && gamepad2.x){
            if(flipDown){
                flipDown = false;
                robot.flipLArm.setPosition(0.1);
                robot.flipRArm.setPosition(0.9);
                dumpTrigger = true;
                timer1.reset();
            } else {
                flipDown = true;
                robot.flipLArm.setPosition(0.5);
                robot.flipRArm.setPosition(0.5);
                robot.dumpFlip.setPosition(0.43);
                downStall = true;
                timer3.reset();
            }
        }

        f = gamepad2.x;

        if(dumpTrigger && timer1.seconds() > 0.5){
            robot.dumpFlip.setPosition(0);
            dumpTrigger = false;
            dumpTrigger2 = true;
        }

        if(dumpTrigger2 && timer1.seconds() > 1.5){
            dumpTrigger2 = false;
            robot.dumpFlip.setPosition(0.1);
        }

        if(downStall && timer3.seconds() > 0.4) {
            downStall = false;
            robot.flipLArm.setPosition(1);
            robot.flipRArm.setPosition(0);
        }
        //endregion

        // region drive the robot code
        rx = Math.pow(gamepad1.right_stick_x, 1);
        ry = -Math.pow(gamepad1.right_stick_y, 1);
        lx = -Math.pow(gamepad1.left_stick_x, 1);
        //endregion

        //region ninja turn
        ninja = gamepad1.right_bumper;
        if(!gamepad1.dpad_left && !gamepad2.dpad_right) {
            if (!ninja) {
                mecanumDrive(rx, ry, lx, 0);
            } else {
                mecanumDrive(rx / 3, ry / 3, lx / 3, 0);
            }
        }
        //endregion

        //region gamepad turn
        if(gamepad1.dpad_right){
            mecanumDrive(0,0,-0.1,0);
        } else if(gamepad1.dpad_left){
            mecanumDrive(0,0,0.1,0);
        }
        //endregion

        // region spine code
        robot.spine.setPower(Range.clip(gamepad2.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger,-1,1));
        //endregion

        //region run intake code

        // region toggle button for intake speed
        if(gamepad2.right_bumper && !iN) {
            if(!inNinja) {
                inNinja = true;
            } else {
                inNinja = false;
            }
        }

        iN = gamepad2.right_bumper;
        //endregion

        double intakePow = gamepad2.right_stick_y;

        if(inNinja) {
            intakePow/=2.4;
        } else {
            intakePow/=1;
        }

        if(intakePow > 0){
            intakePow/= 1.6;
        }


        intakePow = intakePow/0.9;
        robot.intake.setPower(intakePow);
        //endregion

        //region telemetry
        telemetry.addData("pos",pos);
        telemetry.addData("lx", lx);
        telemetry.addData("ry", ry);
        telemetry.addData("rx", rx);
        telemetry.update();
        //endregion
    }

    void mecanumDrive(double lx,double ly,double rx,double k){
        robot.fl.setPower(Range.clip((1+k)*(ly + rx - lx),-1,1));
        robot.bl.setPower(Range.clip((1-k)*(ly + rx + lx),-1,1));
        robot.fr.setPower(Range.clip((1+k)*(ly - rx + lx),-1,1));
        robot.br.setPower(Range.clip((1-k)*(ly - rx - lx),-1,1));
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
