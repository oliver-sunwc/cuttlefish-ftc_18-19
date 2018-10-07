package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.google.blocks.ftcrobotcontroller.util.HardwareType.BNO055IMU;
@Autonomous(name="auttest",group="rover")
public class roverAutoTest extends LinearOpMode {
    roverHMAP robot;
    roverAuto robotAuto;
    String leftColor;
    String rightColor;
    int lwhite = 0;
    int lyellow = 0;
    int rwhite = 0;
    int ryellow = 0;

    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        robot = new roverHMAP();
        robot.init(hardwareMap);
        robotAuto = new roverAuto(robot);
        waitForStart();
        robot.MArmL.setPosition(0.2);
        robot.MArmR.setPosition(0.7);

        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();

        robotAuto.gyroTurnRobotRightAbsolute(43, 0.2);

        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();
    }
}
