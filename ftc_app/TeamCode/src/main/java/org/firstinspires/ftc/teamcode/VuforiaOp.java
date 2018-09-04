
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;


@Autonomous(name="VuforiaThing",group="test")
//@Disabled


public class VuforiaOp extends LinearOpMode {

    BNO055IMU imu;
    Orientation angles;

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorBR;


    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters params;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    String hi;

    private float robotX;
    private float robotY;
    private float robotZ;
    private float robotFirstAngle;
    private float robotSecondAngle;
    private float robotThirdAngle;


    @Override
    public void runOpMode() throws InterruptedException {
        motorBR = hardwareMap.dcMotor.get("BR");
        motorBL = hardwareMap.dcMotor.get("BL");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motorFR.setDirection(DcMotor.Direction.REVERSE);
        //motorBL.setDirection(DcMotor.Direction.REVERSE);

        gamepad1.setJoystickDeadzone((float) 0.05);
        gamepad2.setJoystickDeadzone((float) 0.05);


        // bgyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        startVuforia();

        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();
        visionTargets.activate();

        while (opModeIsActive()) {
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            if (latestLocation != null) {
                lastKnownLocation = latestLocation;
            }

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotZ = coordinates[2];
            robotFirstAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
            robotSecondAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            robotThirdAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            if( Math.pow(Math.pow(robotX,2) + Math.pow(robotY,2) + Math.pow(robotZ,2),0.5) > 200 && RelicRecoveryVuMark.from(target) != RelicRecoveryVuMark.UNKNOWN) {
                vufDrive(0.3);
            } else {
                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
            }

            hi = RelicRecoveryVuMark.from(target).name();
            telemetry.addData("Tracking" + hi, listener.isVisible());
            telemetry.addData("which one", RelicRecoveryVuMark.from(target));
            telemetry.addData("x", robotX);
            telemetry.addData("y", robotY);
            telemetry.addData("z", robotZ);
            telemetry.addData("firstAngle", robotFirstAngle);
            telemetry.addData("secondAngle", robotSecondAngle);
            telemetry.addData("thirdAngle", robotThirdAngle);
            telemetry.update();
        }


    }


    public void startVuforia(){
        params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AQ84BzH/////AAAAGX8oRSEZrUEVouyrxmg33VAWKbOV20N45J7xOEaKgx9eaoZBbWfqB7gKyflDOBewWaCmqM/YCTb8IPOVR1MiqZTfyv2ZRrzZ9E/c0figFfBwHYscqS2s1umHjL9HD8tnsP1g6zrhf0Vn1S/Ht++y7wSQDH3XcAc5QdjUX4AoxSKmcsc5KF3NVkkffXqzrNAZaSZC67V1WDKsLZJMQ2A799tAkAvQ/qEQVYirWsTO346vGix4XAQUYFORJ++64riUrlVmqxXYKkuRCMGkE+CFab2C4faXamnnOeAhwnI2k11g8RKD82xW8QJ7v26hrstjsgQYcoBl0tQy8iCoT3Cvm6Wj8iT6NjjLQt48pPOHiga6";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(params);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");

        target = visionTargets.get(0);
        target.setName("RelicVuMark");
        target.setLocation(createMatrix(0,0,0,0,0,0));

        phoneLocation = createMatrix(0,0,0,0,0,0);

        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation,params.cameraDirection);

    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,u,v,w));
    }

    public String formatMatrix(OpenGLMatrix matrix){
        return matrix.formatAsTransform();
    }
    public void drive(double lx, double ly, double rx){
        motorFL.setPower(Range.clip(0.9*(ly - rx + lx),-1,1));
        motorFR.setPower(Range.clip(0.9*(ly + rx - lx),-1,1));
        motorBL.setPower(Range.clip(1.1*(ly - rx - lx),-1,1));
        motorBR.setPower(Range.clip(1.1*(ly + rx  + lx),-1,1));
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void vufDrive ( double speed) {

        double  max;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // start motion
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motorBL.setPower(speed);
            motorFL.setPower(speed);
            motorFR.setPower(speed);
            motorBR.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (motorBL.isBusy() && motorFR.isBusy() && motorFL.isBusy() && motorBR.isBusy())) {

                // adjust relative speed based on heading error.

                // insert steer define code right here
                steer = -robotX/2000;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    motorBL.setPower(leftSpeed/max);
                    motorFL.setPower(leftSpeed/max);
                    motorBR.setPower(rightSpeed/max);
                    motorFR.setPower(rightSpeed/max);

                } else {
                    motorBL.setPower(leftSpeed);
                    motorFL.setPower(leftSpeed);
                    motorBR.setPower(rightSpeed);
                    motorFR.setPower(rightSpeed);
                }
                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f", steer);
                telemetry.addData("Actual",  "%7d:%7d",         motorBL.getCurrentPosition(),
                        motorBR.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("Tracking" + hi, listener.isVisible());
                telemetry.addData("which one", RelicRecoveryVuMark.from(target));
                telemetry.addData("x", robotX);
                telemetry.addData("y", robotY);
                telemetry.addData("z", robotZ);
                telemetry.addData("firstAngle", robotFirstAngle);
                telemetry.addData("secondAngle", robotSecondAngle);
                telemetry.addData("thirdAngle", robotThirdAngle);
                telemetry.update();
            }

        }
    }
}
