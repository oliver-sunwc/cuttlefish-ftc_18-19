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
    double lx,ly,rx;
    double hang;
    double hangPow = 0.8;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        lx = Math.pow(gamepad1.left_stick_x, 3);
        ly = Math.pow(gamepad1.left_stick_y, 3);
        rx = -Math.pow(gamepad1.right_stick_x, 3);

        if(gamepad2.dpad_up) {
            //robot.hang.setPower(hangPow);
        } else if (gamepad2.dpad_down) {
            //robot.hang.setPower(-hangPow);
        }

        telemetry.addData("lx", lx);
        telemetry.addData("ly", ly);
        telemetry.addData("rx", rx);
        telemetry.update();
        mecanumDrive(lx,ly,rx,0);
    }

    public void mecanumDrive(double lx,double ly,double rx,double k){
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
}
