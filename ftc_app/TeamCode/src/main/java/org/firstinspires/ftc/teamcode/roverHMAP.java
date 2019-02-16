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

public class roverHMAP {

    //Gyro

    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;
    //public AnalogInput landerS;


    double P_DRIVE_COEFF = 0.02;     // Larger is more responsive, but also less stable
    public final double ticksPerInch = 90.3;
    public final double voltage_to_in = 72.2891566265;
    /*Motors*/
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;

    public DcMotor hang;
    public DcMotor spine;

    public DcMotor inFlip;
    public DcMotor intake;

    /*Servos*/
    public Servo flap;
    public Servo flipLArm;
    public Servo flipRArm;
    public Servo rotateArm;

    public DistanceSensor dist;

    HardwareMap hwMap;
    boolean imuInit;

    public roverHMAP() {}

    public void init(HardwareMap ahwMap, boolean imuin) {
        hwMap = ahwMap;
        imuInit = imuin;

        /*Motors*/
        fl = hwMap.get(DcMotor.class, "fl");
        fr = hwMap.get(DcMotor.class, "fr");
        bl = hwMap.get(DcMotor.class, "bl");
        br = hwMap.get(DcMotor.class, "br");

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hang = hwMap.get(DcMotor.class,"h");
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spine = hwMap.get(DcMotor.class,"s");
        spine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        br.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);

        dist = hwMap.get(DistanceSensor.class,"d");

        /*Sensors*/
        //landerS = hwMap.get(AnalogInput.class, "lS");

        /*Servos*/
        flap = hwMap.get(Servo.class, "f");
        flipLArm = hwMap.get(Servo.class, "fLA");
        flipRArm = hwMap.get(Servo.class, "fRA");
        rotateArm = hwMap.get(Servo.class, "rA");

        intake = hwMap.get(DcMotor.class,"i");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        inFlip = hwMap.get(DcMotor.class, "iF");

        if(imuin) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            //parameters.angleUnit = HardwareType.BNO055IMU.AngleUnit.DEGREES;
            //parameters.accelUnit = HardwareType.BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
    }


}
