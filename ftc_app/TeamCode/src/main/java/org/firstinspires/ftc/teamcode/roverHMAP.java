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
    public ColorSensor gS;
    public DistanceSensor mS;

    /*Motors*/
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;

    /*Servos*/
    public Servo MArmL;
    public Servo MArmR;

    HardwareMap hwMap;

    public roverHMAP() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        /*Motors*/
        fL = hwMap.get(DcMotor.class, "FL");
        fR = hwMap.get(DcMotor.class, "FR");
        bL = hwMap.get(DcMotor.class, "BL");
        bR = hwMap.get(DcMotor.class, "BR");

        bR.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.REVERSE);

        /*Servos*/
        MArmL = hwMap.get(Servo.class, "LArm");
        MArmR = hwMap.get(Servo.class, "RArm");

        /*Sensors*/
        gS = hwMap.get(   ColorSensor.class, "gS");
        mS = hwMap.get(DistanceSensor.class, "mS");
    }
}
