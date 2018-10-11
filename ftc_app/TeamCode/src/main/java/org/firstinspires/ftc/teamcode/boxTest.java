package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Manual with Arcade Drive
 */

@Autonomous(name = "boxTest", group = "Rover")
public class boxTest extends LinearOpMode {
    public ColorSensor cs;
    public DistanceSensor ds;
    public Servo s;

    @Override
    public void runOpMode() throws InterruptedException{
        cs = hardwareMap.get(ColorSensor.class, "cs");
        ds = hardwareMap.get(DistanceSensor.class,"cs");
        s = hardwareMap.get(Servo.class,"s");
        s.setPosition(0.4);
        Thread.sleep(2000);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("red", cs.red());
            telemetry.addData("green",cs.green());
            telemetry.addData("blue", cs.blue());
            telemetry.addData("dist",ds.getDistance(DistanceUnit.MM));
            if(ds.getDistance(DistanceUnit.MM) < 500){
                if(cs.red() < 2*cs.blue()) {
                    telemetry.addData("color","white");
                    //s.setPosition(0.8);
                    s.setPosition(0.4);
                } else{
                    telemetry.addData("color","yellow");
                    //s.setPosition(0.3);
                    s.setPosition(0.8);
                }
            } else {
                s.setPosition(0.4);
            }
            telemetry.update();
        }
    }

}
