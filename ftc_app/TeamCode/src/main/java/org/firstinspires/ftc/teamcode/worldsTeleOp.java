package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;
import android.provider.MediaStore;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
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
    boolean dumpTrigger3 = false;

    boolean dumpUp = false;
    boolean override = false;

    boolean transferActive = false;
    boolean k = false;
    @Override
    public void init() {
        robot.init(hardwareMap,false);

        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rotateArm.setPosition(0.5);
        robot.flipLArm.setPosition(1);
        robot.flipRArm.setPosition(0);
        robot.dumpFlip.setPosition(0.35);
        timer1.startTime();
        timer2.startTime();
        timer3.startTime();

        robot.dump.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dump.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dump.setTargetPosition(0);

        robot.dump.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,new PIDCoefficients(5,0,0));

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){

        //region autoTransfer
        if(!k && gamepad2.left_bumper){
            transferActive = true;
        }

        k = gamepad2.left_bumper;
        if(transferActive){
            override = true;
            robot.spine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.spine.setTargetPosition(0);
            robot.spine.setPower(1);
            if(robot.spine.getCurrentPosition() < 200){
                trapDoorUp = false;
                robot.intake.setPower(-1);
            }

        }


        //endregion

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
        if(!i && (gamepad2.right_trigger > 0.1 || gamepad1.left_bumper)){
            inFlipUp = !inFlipUp;
            trapDoorUp = true;
        }

        i = (gamepad2.right_trigger > 0.1 || gamepad1.left_bumper);

        if(inFlipUp){
            robot.inFlip.setPosition(0.15 );
        } else {
            robot.inFlip.setPosition(0.67);
        }
        //endregion

        // region trapDoor toggle
        if(!t && gamepad2.left_trigger > 0.1){
            trapDoorUp = !trapDoorUp;
        }

        t = gamepad2.left_trigger > 0.1;

        if(trapDoorUp){
            robot.trapDoor.setPosition(1);
        } else {
            robot.trapDoor.setPosition(0.13);
        }
        //endregion

        //region dump toggle

        if(!f && gamepad2.x){
            if(dumpUp){
                dumpUp = false;
                robot.dumpFlip.setPosition(1);
                robot.dump.setTargetPosition(0);
                timer2.reset();
                dumpTrigger3 = true;

                robot.rotateArm.setPosition(0.5);

                robot.dump.setPower(0.2);
            } else {
                dumpUp = true;
                robot.dump.setTargetPosition(-1120);
                dumpTrigger = true;
                dumpTrigger2 = true;
                timer1.reset();
                robot.dumpFlip.setPosition(0.1);
                robot.dump.setPower(-0.75);
            }
        }

        if(timer2.seconds() > 0.5 && dumpTrigger3){
            dumpTrigger3 = false;
            robot.dumpFlip.setPosition(0.35);
        }

        f = gamepad2.x;
        if(timer1.seconds() > 0.2 && dumpTrigger){
            robot.dumpFlip.setPosition(0.95);
            dumpTrigger = false;
        }

        if(dumpTrigger2 && timer1.seconds() > 0.35){
            robot.rotateArm.setPosition(0.37);
        }

        if(dumpTrigger2 && robot.dump.getCurrentPosition() < -700){
            robot.dump.setPower(-0.5);
            dumpTrigger2 = false;
        }

        if(gamepad2.y){
            robot.dump.setTargetPosition(-1250);
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
        if(!override) {
            robot.spine.setPower(Range.clip(gamepad2.left_stick_y + gamepad1.left_trigger - gamepad1.right_trigger, -1, 1));
        }
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

        double intakePow = -gamepad2.right_stick_y;

        if(inNinja) {
            intakePow/=2.4;
        } else {
            intakePow/=1;
        }



        if(!override) {
            robot.intake.setPower(intakePow);
        }
        //endregion

        //region telemetry
        telemetry.addData("spine",robot.spine.getCurrentPosition());
        telemetry.addData("pos",pos);
        telemetry.addData("flip",robot.dump.getCurrentPosition());
        telemetry.addData("hangPos", robot.hang.getCurrentPosition());
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
