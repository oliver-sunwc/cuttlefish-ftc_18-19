package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.google.blocks.ftcrobotcontroller.util.HardwareType.BNO055IMU;

public class roverAutoRed extends LinearOpMode {
    roverHMAP robot = new roverHMAP();
    roverAuto robotAuto = new roverAuto();
    public void runOpMode() throws InterruptedException{

        waitForStart();
        robot.hang.setPower(-1);
        Thread.sleep(4000);
        robot.hang.setPower(0);

        robotAuto.gyroDrive(1,21,0);
    }
}
