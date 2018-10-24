package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by howardhuang on 10/6/18.
 */

public class Robot{

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor winch;
    public DcMotor intake; // constantly runs
    public Servo intakeArm;

    static final double AUTO_DRIVE_SPEED_SLOW = 0.25;
    static final double AUTO_DRIVE_SPEED_NORMAL = 0.5;
    static final double AUTO_DRIVE_SPEED_FAST = 1.0;

    /* Initialize Hardware Map */
    HardwareMap hardwareMap =  null;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        // define routes in Hardware Map
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");



        /* Initialize Telemetry */

        // telemetry feedback display
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // reset encoder values
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initiate "getCurrentPosition"

        //int getCurrentPosition();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0","Starting at %7d :%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition());
        telemetry.update();

        // NeverRest Encoders
//    static final double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: NeveRest Motor Encoder
//    static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
//    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    }
}

