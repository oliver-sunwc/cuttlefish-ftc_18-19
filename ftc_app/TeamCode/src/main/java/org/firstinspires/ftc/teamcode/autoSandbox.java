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
    roverAuto robotAuto;

    public void runOpMode() throws InterruptedException {
        robot = new roverHMAP();
        robot.init(hardwareMap);
        robotAuto = new roverAuto(robot);

        waitForStart();

        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() + 500);
        robot.hang.setPower(0.6);
        Thread.sleep(750);
        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() - 1700);
        robot.hang.setPower(-0.3);
        Thread.sleep(1000);
        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() - 1200);
        robot.hang.setPower(-0.7);
        Thread.sleep(1000);

        /*robotAuto.verticalDrive(0.3);
        if(robot.landerS.getVoltage()*robot.voltage_to_in > 6) {
            robotAuto.stopDriving();
        }*/


    }
}

