package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.google.blocks.ftcrobotcontroller.util.HardwareType.BNO055IMU;
@Autonomous(name="autred",group="rover")
public class roverAutoRed extends LinearOpMode {
    roverHMAP robot;
    roverAuto robotAuto;
    AnalogInput ods;
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
        ods = hardwareMap.get(AnalogInput.class, "ods");
        robotAuto = new roverAuto(robot);
        waitForStart();
        //robot.hang.setPower(-1);
        //Thread.sleep(4000);
        //robot.hang.setPower(0);
        robot.MArmL.setPosition(0.72);
        robot.MArmR.setPosition(0.18);
        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();
        robotAuto.verticalDriveDistance(0.1,8.4);
        telemetry.addData("stage one",robotAuto.getHeading());
        telemetry.addData("dl",robot.dMArmL.getDistance(DistanceUnit.CM));
        telemetry.addData("dr",robot.dMArmR.getDistance(DistanceUnit.CM));
        telemetry.update();
        Thread.sleep(500);
        robotAuto.verticalDrive(0.05);

        //run till close
        robotAuto.verticalDrive(0.03);
        do {
            if (robot.dMArmL.getDistance(DistanceUnit.CM) < 6  || robot.dMArmR.getDistance(DistanceUnit.CM)< 6) {
                robotAuto.stopDriving();
                break;
            }

            telemetry.addData("dl",robot.dMArmL.getDistance(DistanceUnit.CM));
            telemetry.addData("dr",robot.dMArmR.getDistance(DistanceUnit.CM));
            telemetry.addData("heading", robotAuto.getHeading());
            telemetry.update();
        } while(true);

        telemetry.addData("stage one", "complete");
        telemetry.addData("heading", robotAuto.getHeading());
        telemetry.update();
        Thread.sleep(250);
        if(Double.isNaN(robot.dMArmL.getDistance(DistanceUnit.CM))){
            robot.fl.setPower(0.02);
            robot.bl.setPower(0.02);
            while(Double.isNaN(robot.dMArmL.getDistance(DistanceUnit.CM))){
                telemetry.addData("dl",robot.dMArmL.getDistance(DistanceUnit.CM));
                telemetry.addData("dr",robot.dMArmR.getDistance(DistanceUnit.CM));
                telemetry.addData("heading", robotAuto.getHeading());
                telemetry.update();
            }
            robot.fl.setPower(0);
            robot.bl.setPower(0);
        }

        if(Double.isNaN(robot.dMArmR.getDistance(DistanceUnit.CM))){
            robot.fr.setPower(0.02);
            robot.br.setPower(0.02);
            while(Double.isNaN(robot.dMArmL.getDistance(DistanceUnit.CM))){
                telemetry.addData("dl",robot.dMArmL.getDistance(DistanceUnit.CM));
                telemetry.addData("dr",robot.dMArmR.getDistance(DistanceUnit.CM));
                telemetry.addData("heading", robotAuto.getHeading());
                telemetry.update();
            }
            robot.fl.setPower(0);
            robot.bl.setPower(0);
        }

        telemetry.addData("stage two", "complete");
        telemetry.addData("heading", robotAuto.getHeading());
        telemetry.update();
        Thread.sleep(250);
        if(robot.dMArmL.getDistance(DistanceUnit.CM) - robot.dMArmR.getDistance(DistanceUnit.CM) > 1.5){
            robot.fl.setPower(0.02);
            robot.bl.setPower(0.02);

            while(robot.dMArmL.getDistance(DistanceUnit.CM) - robot.dMArmR.getDistance(DistanceUnit.CM) > 1.5){

            }
            robot.fl.setPower(0);
            robot.bl.setPower(1);
            robot.fr.setPower(0);
            robot.fl.setPower(0);
        }

        if(robot.dMArmR.getDistance(DistanceUnit.CM) - robot.dMArmL.getDistance(DistanceUnit.CM) > 1.5){
            robot.fr.setPower(0.02);
            robot.br.setPower(0.02);

            while(robot.dMArmR.getDistance(DistanceUnit.CM) - robot.dMArmL.getDistance(DistanceUnit.CM) > 1.5){

            }
            robot.fl.setPower(0);
            robot.bl.setPower(0);
            robot.fr.setPower(0);
            robot.fl.setPower(0);
        }
        telemetry.addData("stage three", "complete");
        telemetry.addData("heading", robotAuto.getHeading());
        telemetry.update();
        Thread.sleep(250);




        telemetry.addData("stage two",2);
        telemetry.addData("heading", robotAuto.getHeading());
        telemetry.update();


        // sense the color
        Thread.sleep(250);
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
            telemetry.addData("heading", robotAuto.getHeading());
            telemetry.update();
            Thread.sleep(50);
        }


        //color sense program
        if(leftColor.equals("white") && rightColor.equals("white")) {
            robot.MArmL.setPosition(0.2);
            robot.MArmR.setPosition(0.7);

        } else if(rightColor.equals("white")) {
            robot.MArmR.setPosition(0.7);
        } else {
            robot .MArmL.setPosition(0.2);

        }
        Thread.sleep(500);
        robotAuto.verticalDriveDistance(0.1,2);
        robot.MArmR.setPosition(0.7);
        robot.MArmL.setPosition(0.2);
        robotAuto.verticalDriveDistance(-0.1,-3.15);
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
