package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Manual with Arcade Drive
 */

@TeleOp(name = "flipTest", group = "Rover")
public class flipTest extends OpMode {


    boolean flipUp, flipControl = false;
    int flipLUpPosition = 400;
    int flipLDownPosition = 0;
    int flipRUpPosition = 400;
    int flipRDownPosition = 0;

    Servo elbowL;
    Servo elbowR;
    DcMotor flipL;
    DcMotor flipR;
    double servoLPos=0;
    double servoRPos=0;
    @Override
    public void init(){
        flipL = hardwareMap.get(DcMotor.class,"fl");
        flipR = hardwareMap.get(DcMotor.class, "fr");

        flipL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flipL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipR.setDirection(DcMotorSimple.Direction.REVERSE);

        elbowL = hardwareMap.get(Servo.class,"eL");
        elbowR = hardwareMap.get(Servo.class,"eR");

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        if(gamepad1.a){
            servoLPos += 0.01;
        }
        if(gamepad1.b){
            servoLPos -= 0.01;
        }
        if(gamepad1.x){
            servoRPos += 0.01;
        }
        if(gamepad1.y){
            servoRPos -=0.01;
        }

        elbowL.setPosition(servoLPos);
        elbowR.setPosition(servoRPos);
        telemetry.addData("elbowL",servoLPos);
        telemetry.addData("elbowR",servoRPos);
    }

}
