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
        telemetry.addData("position:","brake disengaged");
        telemetry.update();
        Thread.sleep(1000);

        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() - 3500);
        robot.hang.setPower(-0.4);
        telemetry.addData("position:","firstDrop");
        telemetry.update();
        Thread.sleep(3000);


        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() - 2000);
        robot.hang.setPower(-0.7);
        telemetry.addData("position:","secondDrop");
        telemetry.update();
        Thread.sleep(2000);

        //gyro align
        robotAuto.verticalDriveDistance(-0.4,-10 );
        //robotAuto.gyroAlign0();



        /*robotAuto.verticalDrive(0.3);
        if(robot.landerS.getVoltage()*robot.voltage_to_in > 6) {
            robotAuto.stopDriving();
        }*/


    }
}

