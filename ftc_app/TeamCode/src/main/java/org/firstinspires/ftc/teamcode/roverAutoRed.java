package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.google.blocks.ftcrobotcontroller.util.HardwareType.BNO055IMU;

public class roverAutoRed extends LinearOpMode {
    roverHMAP robot = new roverHMAP();
    roverAuto robotAuto = new roverAuto();
    String leftColor;
    String rightColor;
    int lwhite = 0;
    int lyellow = 0;
    int rwhite = 0;
    int ryellow = 0;

    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() throws InterruptedException{

        waitForStart();
        robot.hang.setPower(-1);
        Thread.sleep(4000);
        robot.hang.setPower(0);

        robotAuto.verticalDrive(0.4);
        do {
            if (!Double.isNaN(robot.dMArmL.getDistance(DistanceUnit.CM))) {
                robotAuto.stopDriving();
                break;
            }
        } while(true);

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
            sleep(50);
        }
        if(lyellow > lwhite) {
            //set left arm to whack gold
        } else if(ryellow > rwhite) {
            //set right arm to whack gold
        } else {
            //drive forwards to knock gold
        }
        robotAuto.gyroTurnRobotLeft(45);

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
        robotAuto.verticalDriveDistance(-1, -69);
    }
}
