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

    double tempos = 0.5;

    ElapsedTime timer = new ElapsedTime();
    double time;
    @Override
    public void init() {

        robot.init(hardwareMap);
        robot.MArmL.setPosition(0.2);
        robot.MArmR.setPosition(0.7);
        setBox(0.4);

        initialPosition = robot.hang.getCurrentPosition();
    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){

        robot.flipL.setPower(gamepad2.left_stick_y);
        robot.flipR.setPower(-gamepad2.left_stick_y);


        if(gamepad1.left_bumper) {
            tempos += 0.1;
            robot.boxL.setPosition(tempos);
            robot.boxR.setPosition(1.06 - tempos);
        } else if(gamepad1.right_bumper) {
            tempos -= 0.1;
            robot.boxL.setPosition(tempos);
            robot.boxR.setPosition(1.06 - tempos);
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
                setBox(0.1);
                flipUpStage++;
                telemetry.addData("upstage",1);
                timer.reset();
            }

            if(flipUpStage == 1 && timer.seconds()>1) {
                telemetry.addData("upstage",2);
                robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() + 385);
                robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() + 385);
                robot.flipL.setPower(0.2);
                robot.flipR.setPower(0.2);
                flipUpStage++;
                timer.reset();
            }

            if(flipUpStage == 2 && timer.seconds() > 1){
                setBox(1.0/*yeets the claw into megadunk*/);
                flipUpStage++;
            }

            if(flipUpStage == 3){
                flipUpSequence = false;
            }
        }

        if(flipDownSequence){
            if(flipDownStage == 0){
                setBox(0.1);
                flipDownStage++;
                timer.reset();
            }
            if(flipDownStage == 1 && timer.seconds() > 1){
                robot.flipL.setTargetPosition(robot.flipL.getCurrentPosition() - 385);
                robot.flipR.setTargetPosition(robot.flipR.getCurrentPosition() - 385);
                robot.flipL.setPower(-0.2);
                robot.flipR.setPower(-0.2);
                flipDownStage ++;
                timer.reset();
            }
            if(flipDownStage == 2 && timer.seconds() > 2){
                /*yeets the claw forwards, in order to collecc*/
                setBox(0.4);
                flipDownStage++;
            }
            if(flipDownStage == 3){
                flipDownSequence =false;
            }
        }

        if(!flipUpSequence && !flipDownSequence){
            if(secondUp){
                setBox(1.0);
                robot.intakeServo.setPosition(0.5);
            } else {
                setBox(0.4);
                robot.intakeServo.setPosition(1.0);
            }
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
            robot.boxTeeth.setPosition(0.6);
        }

        if(gamepad2.left_stick_y > 0.05){
            robot.intake.setPower(-0.5);
        } else if(gamepad2.left_stick_y < -0.05){
            robot.intake.setPower(0.5);
        } else {
            robot.intake.setPower(0);
        }
        double lx = scaleInput(gamepad1.left_stick_x);

        double ry = scaleInput(-gamepad1.right_stick_y);

        if(dirReversed){
            ry*=-1;
         }
        if(ninja){
            ry/=2;
            lx/=2;
        }
        driveArcade(ry, lx);


        telemetry.addData("boxPos",boxPos);
        telemetry.addData("getPos",robot.boxR.getPosition());
        telemetry.addData("lpos", robot.fl.getCurrentPosition());
        telemetry.addData("rpos", robot.fr.getCurrentPosition());
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
