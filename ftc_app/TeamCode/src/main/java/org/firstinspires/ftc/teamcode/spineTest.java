package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;


/**
 * Created by Lenovo on 8/6/2018.
 */

@TeleOp(name = "spineTest")
@Disabled
public class spineTest extends OpMode {
    roverHMAP robot = new roverHMAP();
    double rx,ry,lx;
    double hang;
    double hangPow = 1;

    boolean fliptog = false;
    boolean f;
    boolean rotatetog = false;
    boolean r;
    boolean ninja = false;
    boolean n;
    int intakeTog = 0;
    boolean i;
    int flipAng = 2500;
    boolean inNinja = false;
    boolean iN;
    boolean flapTog = false;
    boolean fL;

    int initialPosition;
    ElapsedTime flipTimer = new ElapsedTime();

    @Override
    public void init(){
        robot.init(hardwareMap,true);
        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initialPosition = robot.hang.getCurrentPosition();
    }

    @Override
    public void loop(){
        if(!f && gamepad1.x) {
            if(!fliptog) {
                robot.flipLArm.setPosition(1);
                robot.flipRArm.setPosition(0);
                flipTimer.reset();
                fliptog = true;
            } else {
                robot.flipLArm.setPosition(0);
                robot.flipRArm.setPosition(1);
                flipTimer.reset();
                fliptog = false;
            }
        }
        f = gamepad1.x;

        if(fliptog && flipTimer.milliseconds() >= 300) {
            robot.rotateArm.setPosition(0.5);
        } else if(!fliptog && flipTimer.milliseconds() >= 300) {
            robot.rotateArm.setPosition(0);
        }

        if(!r && gamepad1.y) {
            if(!rotatetog) {
                rotatetog = true;
                robot.flipLArm.setPosition(0.1);
                robot.flipRArm.setPosition(0.9);
            } else {
                robot.flipLArm.setPosition(1);
                robot.flipRArm.setPosition(0);
                rotatetog = false;
            }
        }

        r = gamepad1.y;

        telemetry.addData("rotate toggle", rotatetog);
        telemetry.addData("flip toggle", fliptog);
    }
}
