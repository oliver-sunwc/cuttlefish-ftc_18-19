package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.*;

/**
 * Manual with Arcade Drive
 */

@TeleOp(name = "TankFinal", group = "Rover")
public class roverTeleOP extends OpMode {

    roverHMAP robot = new roverHMAP();
    boolean ninja, dirReversed, dirControl = false;
    boolean dirToggle = false;
    double lx, rx, ly, ry;

    boolean flipUp, flipControl = false;
    double boxPos = 1;
    boolean boxUpControl, boxDownControl = false;
    boolean  intakeControl = false;
    boolean intakeUp = true;

    boolean flipUpSequence, flipDownSequence = false;
    int flipUpStage, flipDownStage = 0;

    ElapsedTime motorTime = new ElapsedTime();
    boolean flapUp, flapControl = false;
    boolean teethUp, teethControl = false;
    int initialPosition;

    boolean secondUp, secondUpControl = false;

    int relativeTicks;
    double tempos = 0.5;

    ElapsedTime timer = new ElapsedTime();
    double time;
    @Override
    public void init() {

        robot.init(hardwareMap);
    }
    @Override
    public void start(){
        robot.MArmL.setPosition(0.2);
        robot.MArmR.setPosition(0.8);
        boxPos=0.45;
        setBox(boxPos);
        relativeTicks = robot.flipL.getCurrentPosition() + 75;
        robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() + 75);
        robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() + 75);
        robot.flipL.setPower(0.2);
        robot.flipR.setPower(0.2);
        initialPosition = robot.hang.getCurrentPosition();


    }

    @Override
    public void loop(){
        if(Math.abs(robot.flipL.getCurrentPosition() - relativeTicks) < 10) {
            robot.flipL.setPower(0);
            robot.flipR.setPower(0);
        }

        if(!gamepad2.right_bumper){
            secondUpControl = true;
        }

        if(gamepad2.right_bumper && secondUpControl){
            secondUp = !secondUp;
            secondUpControl = false;
        }

        if(gamepad1.right_bumper){
            ninja= true;
        } else {
            ninja = false;
        }
        // toggle buttons
        if(!gamepad1.b){
            dirControl = true;
        }

        if(gamepad1.b && dirControl){
            dirReversed = !dirReversed;
            dirControl = false;
        }
        if(!gamepad2.a){
            flipControl = true;
        }


        if(gamepad2.a && flipControl) {
            if (flipUp) {
                if(!flipUpSequence) {
                    flipDownSequence = true;
                    flipDownStage = 0;
                }

                flipControl = false;
                flipUp = false;
            } else {
                if(!flipDownSequence) {
                    flipUpSequence = true;
                    flipUpStage = 0;
                }

                flipControl = false;
                flipUp = true;
            }
        }

        if(!gamepad2.y){
            flapControl = true;
        }

        if(gamepad2.y && flapControl){
            flapUp = !flapUp;
            flapControl = false;
        }

        if(!gamepad2.b){
            teethControl = true;
        }

        if(gamepad2.b && teethControl){
            teethUp = !teethUp;
            teethControl = false;
        }

        if(flipUpSequence){
            if(flipUpStage == 0) {
                /*yeets the box back, in order to move it out of the bar's way*/
                boxPos = 0.3;
                flipUpStage++;
                telemetry.addData("upstage",1);
                timer.reset();
            }

            if(flipUpStage == 1 && timer.seconds()>0.4) {
                telemetry.addData("upstage",2);
                robot.flipL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                relativeTicks = robot.flipL.getCurrentPosition() + 400;
                robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() + 400);
                robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() + 400);
                robot.flipL.setPower(0.2);
                robot.flipR.setPower(0.2);
                flipUpStage++;
                timer.reset();
            }

            if(flipUpStage == 2 && timer.seconds() > 1){
                boxPos = 0.7;
                flipUpStage++;
            }

            if(flipUpStage == 3){
                flipUpSequence = false;
            }
        }

        if(flipDownSequence){
            if(flipDownStage == 0){
                boxPos = 0.23;
                flipDownStage++;
                timer.reset();
            }
            if(flipDownStage == 1 && timer.seconds() > 1){
                relativeTicks = robot.flipL.getCurrentPosition() - 300;
                robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() - 300);
                robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() - 300);
                robot.flipL.setPower(-0.14);
                robot.flipR.setPower(-0.14);
                flipDownStage ++;
                timer.reset();
            }
            if(flipDownStage == 2 && timer.seconds() > 1){
                /*yeets the claw forwards, in order to collecc*/
                boxPos = 0.45;
                relativeTicks = robot.flipL.getCurrentPosition() - 100;
                robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() - 100);
                robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() - 100);
                robot.flipL.setPower(-0.14);
                robot.flipR.setPower(-0.14);

                flipDownStage++;
                timer.reset();
            }

            if(flipDownStage == 3){
                flipDownSequence =false;
            }
        }

        if(!flipUpSequence && !flipDownSequence){

        }
        if(!gamepad2.x){
            intakeControl = true;
        }

        if(gamepad2.x && intakeControl){
            intakeControl = false;
            intakeUp = !intakeUp;
        }


        if(gamepad2.dpad_up){
            robot.hang.setPower(1.0);
            initialPosition += 250;

        } else if(gamepad2.dpad_down){
            robot.hang.setPower(-1.0);
            initialPosition -= 250;
        } else{
            robot.hang.setTargetPosition(initialPosition);
        }
        robot.hang.setTargetPosition(initialPosition);
        if(robot.hang.getCurrentPosition() - 30 < initialPosition){
            robot.hang.setPower(1.0);
        } else if (robot.hang.getCurrentPosition() + 30 > initialPosition) {
            robot.hang.setPower(-1.0);
        }



        if(flapUp){
            robot.boxFlap.setPosition(0.1);
        } else {
            robot.boxFlap.setPosition(0.6);
        }

        if(teethUp){
            robot.boxTeeth.setPosition(0.1);
        } else {
            robot.boxTeeth.setPosition(0.69);
        }

        if(gamepad2.left_stick_y > 0.05){
            robot.intake.setPower(-0.5);
        } else if(gamepad2.left_stick_y < -0.05){
            robot.intake.setPower(0.5);
        } else {
            robot.intake.setPower(0);
        }


        if(!secondUp) {
            if (intakeUp) {
                robot.intakeServo.setPosition(0.3);
            } else {
                robot.intakeServo.setPosition(0.84);
            }

            setBox(boxPos);
        } else {
            robot.intakeServo.setPosition(0.8);
            setBox(0.9);
        }



        double lx = scaleInput(gamepad1.left_stick_x);

        double ry = scaleInput(-gamepad1.right_stick_y);

        if(dirReversed){
            ry*=-1;
         }
        if(ninja){
            ry/=7;
            lx/=7;
        }
        driveArcade(ry, lx);


        telemetry.addData("boxPos",boxPos);
        telemetry.addData("getPos",robot.boxR.getPosition());
        telemetry.addData("ninja", ninja);
        telemetry.addData("direction", dirReversed);
        telemetry.addData("lpos", robot.flipL.getCurrentPosition());
        telemetry.addData("rpos", robot.flipR.getCurrentPosition());
        telemetry.update();
    }

    void driveTank(double ly, double ry) {
        robot.fl.setPower(ly);
        robot.bl.setPower(ly);
        robot.fr.setPower(ry);
        robot.br.setPower(ry);

        if(ly != 0 || ry != 0) {
            motorTime.reset();
        } else {
            time = motorTime.seconds()*1.0;
            String filename = "MotorLife.json";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, "motor life " + Double.toString(time));

            telemetry.log().add("saved to '%s'", filename);
        }

        telemetry.addData("dl",robot.dMArmL.getDistance(DistanceUnit.CM));
        telemetry.addData("dl",robot.dMArmL.getDistance(DistanceUnit.CM));
        telemetry.update();

    }

    void setBox(double pos){
        if(0 <= pos && pos <= 1.0) {
            robot.boxR.setPosition(1.06 - pos);
            robot.boxL.setPosition(pos);
        }
    }

    void driveArcade(double ry, double lx){
        robot.fl.setPower(ry + lx);
        robot.bl.setPower(ry + lx);
        robot.fr.setPower(ry - lx);
        robot.br.setPower(ry - lx);
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
}
