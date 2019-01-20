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
    int flipAng = 2500;
    boolean inNinja = false;
    boolean iN;
    boolean flapTog = false;
    boolean fL;

    ElapsedTime stallTime = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){

        //toggle button for intake speed
        if(gamepad2.right_bumper && !iN) {
            if(!inNinja) {
                inNinja = true;
            } else {
                inNinja = false;
            }
        }

        iN = gamepad2.right_bumper;

        //intake flip toggle position
        if((gamepad2.a || gamepad1.left_bumper) && !i) {
            if(intakeTog == 0) {
                intakeTog = 1;
                robot.inFlip.setPower(0.9);
                robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 110);
            } else if(intakeTog == 1) {
                intakeTog = 0;
                robot.inFlip.setPower(-0.9);
                robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() - 110);            }
        }

        i = (gamepad2.a || gamepad1.left_bumper);

        // do based on intake position


        //flip toggle position
        if(!f && gamepad2.x) {
            if (!fliptog) {
                fliptog = true;
                robot.flipLArm.setPosition(1);
                robot.flipRArm.setPosition(0);
            } else {
                fliptog = false;
                robot.flipLArm.setPosition(0);
                robot.flipRArm.setPosition(1);
            }
        }

        f = gamepad2.x;

        // rotate toggle position
        if(!r && gamepad2.y) {
            if(!rotatetog) {
                rotatetog = true;
                robot.rotateArm.setPosition(0.50);
            } else {
                rotatetog = false;
                robot.rotateArm.setPosition(0);
            }
        }

        r = gamepad2.y;



        // ninja thing
        ninja = gamepad1.right_bumper;

        // drive the robot code
        if(gamepad1.right_stick_y != 0 || gamepad1.right_stick_x == 0) {
            rx = Math.pow(gamepad1.right_stick_x, 1);
        } else {
            rx += gamepad1.right_stick_x/250;
        }
        ry = -Math.pow(gamepad1.right_stick_y, 1);
        lx = -Math.pow(gamepad1.left_stick_x, 1);

        if(!fL && gamepad2.b) {
            if(!flapTog) {
                flapTog = true;
                robot.flap.setPosition(0.9);
            } else {
                flapTog = false;
                robot.flap.setPosition(0.1);
            }
        }

        fL = gamepad2.b;

        if(!gamepad1.dpad_left && !gamepad2.dpad_right) {
            if (!ninja) {
                mecanumDrive(rx, ry, lx, 0);
            } else {
                mecanumDrive(rx / 3, ry / 3, lx / 3, 0);
            }
        } else if(gamepad1.dpad_left) {
            mecanumDrive(0.0,0,-0.2,0);
        } else if(gamepad1.dpad_right) {
            mecanumDrive(0.0,0,0.2,0);
        }
        // hang code
        if(gamepad2.dpad_up) {
            robot.hang.setPower(-hangPow);
            robot.hang.setTargetPosition(robot.hang.getCurrentPosition() - 200);

        } else if (gamepad2.dpad_down) {
            robot.hang.setPower(hangPow);
            robot.hang.setTargetPosition(robot.hang.getCurrentPosition() + 200);
        } else {
            //robot.hang.setPower(0);
        }

        // spine code
        robot.spine.setPower(gamepad2.left_stick_y);

        //run intake code
        if(inNinja) {
            robot.intake.setPower(gamepad2.right_stick_y/3.2);
        } else {
            robot.intake.setPower(gamepad2.right_stick_y/1.6);
        }

        if(gamepad2.left_trigger != 0.1 && gamepad2.right_trigger > 0.1) {
            hangStall();
        }

        telemetry.addData("lx", lx);
        telemetry.addData("ry", ry);
        telemetry.addData("rx", rx);
        telemetry.addData("thing",robot.inFlip.getCurrentPosition());
        telemetry.update();

    }

    void mecanumDrive(double lx,double ly,double rx,double k){
        robot.fl.setPower(Range.clip((1+k)*(ry + rx - lx),-1,1));
        robot.bl.setPower(Range.clip((1-k)*(ry + rx + lx),-1,1));
        robot.fr.setPower(Range.clip((1+k)*(ry - rx + lx),-1,1));
        robot.br.setPower(Range.clip((1-k)*(ry - rx - lx),-1,1));
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
}
