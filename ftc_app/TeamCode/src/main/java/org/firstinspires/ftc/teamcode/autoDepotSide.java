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

@Autonomous(name="autDepot")
public class autoDepotSide extends LinearOpMode {
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
        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() + 500);
        robot.hang.setPower(0.6);
        telemetry.addData("position:","brake disengaged");
        telemetry.update();
        Thread.sleep(5000);

        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() - 3500);
        robot.hang.setPower(-0.4);
        telemetry.addData("position:","firstDrop");
        telemetry.update();
        Thread.sleep(5000);


        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() - 1000);
        robot.hang.setPower(-0.7);
        telemetry.addData("position:","secondDrop");
        telemetry.update();
        Thread.sleep(5000);


        vision.disable();


    }
}

