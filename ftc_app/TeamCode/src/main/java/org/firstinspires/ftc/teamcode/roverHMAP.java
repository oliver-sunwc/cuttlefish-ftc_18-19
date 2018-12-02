package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Created by oliversun on 10/7/17.
 */

public class roverHMAP{

    //Gyro

    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;

    /*Sensors*/
    public DistanceSensor dRight;
    public DistanceSensor dBack;

    double P_DRIVE_COEFF = 0.02;     // Larger is more responsive, but also less stable
    public final double ticksPerInch = 72.1;
    /*Motors*/
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;

    public DcMotor flipL;
    public DcMotor flipR;
    public DcMotor hang;
    public DcMotor intake;
    /*Servos*/
    public Servo intakeDrop;
    public Servo gameMarker;

    HardwareMap hwMap;

    public roverHMAP() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;


        /*Motors*/
        fl = hwMap.get(DcMotor.class, "fl");
        fr = hwMap.get(DcMotor.class, "fr");
        bl = hwMap.get(DcMotor.class, "bl");
        br = hwMap.get(DcMotor.class, "br");

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //flip = hwMap.get(DcMotor.class,"f");
        //hang = hwMap.get(DcMotor.class,"hl");
        //intake = hwMap.get(DcMotor.class,"i");


        br.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);


        //boxClose = hwMap.get(Servo.class, "cb");
        //boxRotate = hwMap.get(Servo.class, "rotb");

        /*Sensors*/
        dRight = hwMap.get(DistanceSensor.class,"dr");
        dBack = hwMap.get(DistanceSensor.class, "db");

        gameMarker = hwMap.get(Servo.class,"gm");

        flipL = hwMap.get(DcMotor.class,"fll");
        flipR = hwMap.get(DcMotor.class,"flr");


        flipL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flipL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        flipL.setDirection(DcMotorSimple.Direction.REVERSE);

        flipL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeDrop = hwMap.get(Servo.class,"s");
        intake = hwMap.get(DcMotor.class,"i");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hang = hwMap.get(DcMotor.class,"h");
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit = HardwareType.BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit = HardwareType.BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }


}
