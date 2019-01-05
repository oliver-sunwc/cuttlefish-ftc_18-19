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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name="autoSandbox")
public class autoSandbox extends LinearOpMode {
    roverHMAP robot;

    public void runOpMode() throws InterruptedException {
        robot = new roverHMAP();
        robot.init(hardwareMap);

        waitForStart();


    }

    void stopDriving() {
        robot.fl.setPower(0);
        robot.fr.setPower(0);
        robot.bl.setPower(0);
        robot.br.setPower(0);
    }

    void verticalDrive(double power) {
        robot.fl.setPower(power);
        robot.fr.setPower(power);
        robot.bl.setPower(power);
        robot.br.setPower(power);
    }

    void rotateRight(double power) {
        robot.fl.setPower(-power);
        robot.bl.setPower(-power);
        robot.fr.setPower(power);
        robot.br.setPower(power);
    }

    void rotateLeft(double power) {
        rotateRight(-power);
    }

    void verticalDriveDistance(double power, double distance) throws InterruptedException {
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        distance = (int)(distance*robot.ticksPerInch);
        int flDist = robot.fl.getCurrentPosition();
        int frDist = robot.fr.getCurrentPosition();
        int blDist = robot.bl.getCurrentPosition();
        int brDist = robot.br.getCurrentPosition();
        verticalDrive(power);

        if(distance > 0) {
            while (robot.fl.getCurrentPosition() - flDist < distance &&
                    robot.fr.getCurrentPosition() - frDist< distance &&
                    robot.bl.getCurrentPosition() - blDist< distance &&
                    robot.br.getCurrentPosition() - brDist< distance) {
            }
        } else {
            while (robot.fl.getCurrentPosition() > distance &&
                    robot.fr.getCurrentPosition() > distance &&
                    robot.bl.getCurrentPosition() > distance &&
                    robot.br.getCurrentPosition() > distance) {
            }
        }

        stopDriving();
    }

    void RotateDistance(double power, int distance) throws InterruptedException {
        {
            //reset encoders
            robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.fl.setTargetPosition(distance);
            robot.fr.setTargetPosition(-distance);
            robot.bl.setTargetPosition(distance);
            robot.br.setTargetPosition(-distance);

            robot.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rotateRight(power);

            while (robot.fl.isBusy() && robot.fr.isBusy() && robot.bl.isBusy() && robot.br.isBusy()) {
                //wait until robot stops
            }

            stopDriving();
        }
    }
}

