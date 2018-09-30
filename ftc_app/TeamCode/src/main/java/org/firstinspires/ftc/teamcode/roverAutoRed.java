package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.google.blocks.ftcrobotcontroller.util.HardwareType.BNO055IMU;
@Autonomous(name="autred",group="rover")
public class roverAutoRed extends LinearOpMode {
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
        //robot.hang.setPower(-1);
        //Thread.sleep(4000);
        //robot.hang.setPower(0);
        robot.MArmL.setPosition(0.76);
        robot.MArmR.setPosition(0.18);
        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();
        robotAuto.verticalDriveDistance(0.1,9.25);
        telemetry.addData("stage one",1);
        telemetry.update();
        Thread.sleep(500);
        robotAuto.verticalDrive(0.05);
        do {
            if (!Double.isNaN(robot.dMArmL.getDistance(DistanceUnit.CM))  || !(Double.isNaN(robot.dMArmR.getDistance(DistanceUnit.CM)))) {
                robotAuto.stopDriving();
                break;
            }

            telemetry.addData("dl",robot.dMArmL.getDistance(DistanceUnit.CM));
            telemetry.addData("dr",robot.dMArmR.getDistance(DistanceUnit.CM));
            telemetry.update();
        } while(true);


        telemetry.addData("stage two",2);
        telemetry.update();
        timer.reset();
        while(timer.seconds() < 1) {
            if (robot.cMArmL.red() < 2 * robot.cMArmL.blue()) {
                leftColor = "white";
                lwhite++;
            } else {
                leftColor = "yellow";
                lyellow++;
            }
            telemetry.addData("left color", leftColor);
            if (robot.cMArmR.red() < 2 * robot.cMArmR.blue()) {
                rightColor = "white";
                lwhite++;
            } else {
                rightColor = "yellow";
                lyellow++;
            }
            telemetry.addData("right color", rightColor);
            telemetry.update();
            Thread.sleep(50);
        }
        if(leftColor.equals("white") && rightColor.equals("white")) {
            robot.MArmL.setPosition(0.2);
            robot.MArmR.setPosition(0.7);

        } else if(rightColor.equals("white")) {
            robot.MArmR.setPosition(0.7);
        } else {
            robot.MArmL.setPosition(0.2);

        }
        Thread.sleep(500);
        robotAuto.verticalDriveDistance(0.1,2);
        robot.MArmR.setPosition(0.7);
        robot.MArmL.setPosition(0.2);
        robotAuto.verticalDriveDistance(-0.1,-3.5);
        robotAuto.gyroTurnRobotLeft(90,0.1);
        robotAuto.verticalDriveDistance(0.1,24);
        robotAuto.gyroTurnRobotLeftAbsolute(-90,0.1);
        /*
        robotAuto.gyroDrive(0.4, 69, -45);

        double rightTurnHeading = robotAuto.getHeading();
        if(robotAuto.getHeading() >= -180 && robotAuto.getHeading() <= -180+(90+5)){
            robot.br.setPower(0.2);
            robot.fr.setPower(0.2);
            while(robotAuto.normalize(robotAuto.getHeading()) < robotAuto.normalize(rightTurnHeading) + (90-3)){
            }
            robot.br.setPower(0);
            robot.fr.setPower(0);

        } else {
            robot.br.setPower(0.2);
            robot.fr.setPower(0.2);
            while(robotAuto.getHeading() < rightTurnHeading + (90-3)){
            }
            robot.fr.setPower(0);
            robot.fl.setPower(0);
        }

        robotAuto.verticalDriveDistance(1, 69);
        //yeet the marker
        robotAuto.verticalDriveDistance(-1, -69);*/
    }
}
