    package org.firstinspires.ftc.teamcode;

        import android.graphics.Color;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
        import com.qualcomm.robotcore.hardware.AnalogInput;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.DistanceSensor;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.CRServo;

        import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

    /*Sensors*/

    //This is the sensor that will sense the mineral to be knocked off the starting zone
    public ColorSensor cMArmL;
    public DistanceSensor dMArmL;

    public ColorSensor cMArmR;
    public DistanceSensor dMArmR;

    /*Motors*/
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;

    public DcMotor flip;
    public DcMotor hangLeft;
    public DcMotor hangRight;
    public DcMotor intake;
    /*Servos*/
    public Servo MArmL;
    public Servo MArmR;

    public Servo boxClose;
    public Servo boxLeft;
    public Servo boxRight;
    public Servo boxRotate;

    HardwareMap hwMap;

    public roverHMAP() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        /*Motors*/
        fl = hwMap.get(DcMotor.class, "fl");
        fr = hwMap.get(DcMotor.class, "fr");
        bl = hwMap.get(DcMotor.class, "bl");
        br = hwMap.get(DcMotor.class, "br");

        flip = hwMap.get(DcMotor.class,"f");
        hangLeft = hwMap.get(DcMotor.class,"hl");
        hangRight = hwMap.get(DcMotor.class,"hr");
        intake = hwMap.get(DcMotor.class,"i");


        br.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);

        /*Servos*/
        MArmL = hwMap.get(Servo.class, "larm");
        MArmR = hwMap.get(Servo.class, "rarm");

        boxClose = hwMap.get(Servo.class, "cb");
        boxLeft = hwMap.get(Servo.class, "lb");
        boxRight = hwMap.get(Servo.class, "rb");
        boxRotate = hwMap.get(Servo.class, "rotb");

        /*Sensors*/
        cMArmL = hwMap.get(   ColorSensor.class, "cL");
        dMArmL = hwMap.get(DistanceSensor.class, "cL");

        cMArmR = hwMap.get(   ColorSensor.class, "cR");
        dMArmR = hwMap.get(DistanceSensor.class, "cR");
    }
}
