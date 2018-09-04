package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.DistanceSensor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name = "Swerve")

public class Swerve extends OpMode {
    BNO055IMU imu;
    Orientation angles;

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;

    Servo servoFR;
    Servo servoFL;
    Servo servoBR;
    Servo servoBL;
    double fwd,str,rot;
    double wb,tw,r;//wheelbase and trackwidth

    boolean aControl,isField;

    @Override
    public void init(){
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorBL = hardwareMap.dcMotor.get("BL");

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoFR = hardwareMap.servo.get("sFR");
        servoFL = hardwareMap.servo.get("sFL");
        servoBR = hardwareMap.servo.get("sBR");
        servoBL = hardwareMap.servo.get("sBL");

        servoFR.setPosition(0.5);
        servoFL.setPosition(0.5);
        servoFR.setPosition(0.5);
        servoBL.setPosition(0.5);
        gamepad1.setJoystickDeadzone((float)0.05);
        gamepad2.setJoystickDeadzone((float)0.05);
        wb = 1;
        tw = 1;
        r = Math.sqrt(2);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        aControl = false;
        isField = false;
    }

    @Override
    public void loop(){
        /*if(gamepad1.a){
            aControl = true;
        }

        if(!gamepad1.a && aControl){
            isField = !isField;
            aControl = false;
        }*/


        fwd = -gamepad1.right_stick_y;
        str = gamepad1.right_stick_x;
        rot = -gamepad1.left_stick_x;

        /*if(isField) {
            double temp = fwd * Math.cos(normalize(getHeading()) * Math.PI / 180) + str * Math.sin(normalize(getHeading()) * Math.PI / 180);
            str = -fwd * Math.sin(normalize(getHeading()) * Math.PI / 180) + str * Math.cos(normalize(getHeading()) * Math.PI / 180);
            fwd = temp;
        }*/

        double a = str - rot*(wb/r);
        double b = str + rot*(wb/r);
        double c = fwd - rot*(tw/r);
        double d = fwd + rot*(tw/r);

        double wsfr = Math.sqrt(b*b + c*c);
        double wsfl = Math.sqrt(b*b + d*d);
        double wsbl = Math.sqrt(a*a + d*d);
        double wsbr = Math.sqrt(a*a + c*c);

        double wafr = Math.atan2(b,c)*180/Math.PI;
        double wafl = Math.atan2(b,d)*180/Math.PI;
        double wabl = Math.atan2(a,d)*180/Math.PI;
        double wabr = Math.atan2(a,c)*180/Math.PI;

        double max = wsfr;
        if(wsfl > max){
            max = wsfl;
        }
        if(wsbr > max){
            max = wsbr;
        }
        if(wsbl > max){
            max = wsbl;
        }

        if(max>1){
            wsfr/=max;
            wsfl/=max;
            wsbr/=max;
            wsbl/=max;
        }

        //the variables ws... and wa... are the corresponding values for the speed and angles: now we need
        // a control loop to convert these into values: for reference: 0<=ws<=1 && -180<=wa<=180(in degrees) and 0 is straight ahead
        if(wafr <= -90) {
            wafr += 180;
            wsfr*=-1;
        }
        if(wafr >= 90) {
            wafr -= 180;
            wsfr*=-1;
        }


        if(wafl <= -90) {
            wafl += 180;
            wsfl*=-1;
        }
        if(wafl >= 90) {
            wafl -= 180;
            wsfl*=-1;
        }


        if(wabr <= -90) {
            wabr += 180;
            wsbr*=-1;
        }
        if(wabr >= 90) {
            wabr -= 180;
            wsbr*=-1;
        }


        if(wabl <= -90) {
            wabl += 180;
            wsbl*=-1;
        }
        if(wabl >= 90) {
            wabl -= 180;
            wsbl*=-1;
        }

        // now the angles are normalized into -90 to 90 and the motor power is from -1 to 1

        servoFR.setPosition(1-(wafr + 90)/180);
        servoFL.setPosition(1-(wafl + 90)/180);
        servoBR.setPosition(1-(wabr + 90)/180);
        servoBL.setPosition(1-(wabl + 90)/180);

        motorFR.setPower(wsfl);
        motorFL.setPower(wsfr);
        motorBR.setPower(wsbl);
        motorBL.setPower(wsbr);

        telemetry.addData("wsfl",wsfl);
        telemetry.addData("wsfr",wsfr);
        telemetry.addData("wsbl",wsbl);
        telemetry.addData("wsbr",wsbr);
        telemetry.addData("wafl",wafl);
        telemetry.addData("wafr",wafr);
        telemetry.addData("wabl",wabl);
        telemetry.addData("wabr",wabr);
        telemetry.update();
    }

    public double normalize(double hi){
        if(hi < 0){
            hi = -hi;
            hi = 180-hi;
            hi += 180;
        }
        return hi;
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }
}
