package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name="autCrater")
public class autoCraterSide extends LinearOpMode {
    roverHMAP robot;
    roverAuto robotAuto;
    private VisionThing vision;

    public void runOpMode() throws InterruptedException {
        robot = new roverHMAP();
        robot.init(hardwareMap);
        robotAuto = new roverAuto(robot);

        vision = new VisionThing();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();

        waitForStart();
        //robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //robot.hang.setTargetPosition(robot.hang.getCurrentPosition() + 700);
        robot.hang.setPower(0.8);
        telemetry.addData("position:","brake disengaged");
        telemetry.update();
        Thread.sleep(500);
        robot.hang.setPower(0);
        sleep(1000);

        //robot.hang.setTargetPosition(robot.hang.getCurrentPosition() - 7000);
        robot.hang.setPower(-1);
        telemetry.addData("position:","firstDrop");
        telemetry.update();
        Thread.sleep(1500);
        robot.hang.setPower(0);

        /*robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() - 7000);
        robot.hang.setPower(-0.9);
        telemetry.addData("position:","secondDrop");
        telemetry.update();*/

        Thread.sleep(1000);
        robotAuto.verticalDriveDistance(0.4,1 );
        //gyro align

        //robot.hang.setTargetPosition(robot.hang.getCurrentPosition());
        //robot.hang.setPower(0);

        Thread.sleep(1000);
        telemetry.addData("dist",robot.dist.getDistance(DistanceUnit.CM));
        telemetry.update();

        robotAuto.verticalDriveDistance(0.1,5);

         // gyro align

        robotAuto.moveForward(0.2);
        while(robot.dist.getDistance(DistanceUnit.CM) < 25){
            telemetry.addData("dist",robot.dist.getDistance(DistanceUnit.CM));
            telemetry.update();

        }

        robotAuto.stopDriving();
        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();
        Thread.sleep(2000);

        // do vision thing
        while(robotAuto.getHeading() > -90){
            telemetry.addData("gyro",robotAuto.getHeading());
            telemetry.update();
            robot.fl.setPower(0.2);
            robot.bl.setPower(0.2);
            robot.br.setPower(-0.2);
            robot.fr.setPower(-0.2);
        }
        robotAuto.stopDriving();

        telemetry.addData("gyro", robotAuto.getHeading());
        telemetry.update();
        sleep(50000);

        //align with sensor
        /*robotAuto.verticalDrive(0.3);
        if(robot.landerS.getVoltage()*robot.voltage_to_in > 6) {
            robotAuto.stopDriving();
        }*/

        /*
        extend slide and drop intake loop
         */

        /*
        intake the cube
        */
        vision.disable();


    }

    void checkStop(){
        if(isStopRequested()){
            vision.disable();
        }
    }
}

