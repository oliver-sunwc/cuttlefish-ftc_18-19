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


    ElapsedTime motorTime = new ElapsedTime();

    int initialPosition;



    double time;
    @Override
    public void init() {

        robot.init(hardwareMap);
    }
    @Override
    public void start(){

    }

    @Override
    public void loop(){




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


        if(gamepad2.left_stick_y > 0.05){
            robot.intake.setPower(-0.5);
        } else if(gamepad2.left_stick_y < -0.05){
            robot.intake.setPower(0.5);
        } else {
            robot.intake.setPower(0);
        }

        robot.flipL.setPower(-gamepad2.right_stick_y);
        robot.flipR.setPower(gamepad2.right_stick_y);



        double ry = scaleInput(-gamepad1.right_stick_y);

        if(dirReversed){
            ry*=-1;
         }

        if(ninja){
            ry/=7;
            lx/=7;
        }
        driveArcade(ry, lx);



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




    void driveArcade(double ry, double lx){
        robot.fl.setPower(ry/2 + lx);
        robot.bl.setPower(ry/2 + lx);
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
