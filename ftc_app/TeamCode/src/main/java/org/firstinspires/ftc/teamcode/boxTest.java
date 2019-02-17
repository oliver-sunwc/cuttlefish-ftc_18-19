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

@TeleOp(name = "BoxTest")
@Disabled
public class boxTest extends OpMode {
    DcMotor box;
    DcMotor joint;
    boolean ninja = false;
    boolean n = false;

    @Override
    public void init(){
        box = hardwareMap.get(DcMotor.class, "box");
        joint = hardwareMap.get(DcMotor.class, "joint");
    }

    @Override
    public void loop(){

        if(!n && gamepad1.a) {
            if (!ninja) {
                ninja = true;
            } else {
                ninja = false;
            }
        }

        n = gamepad1.a;

        if(!ninja) {
            box.setPower(gamepad1.left_stick_y);
            telemetry.addData("ninja", ninja);
            telemetry.addData("speed", gamepad1.left_stick_y);
            telemetry.addData("n", n);
            telemetry.addData("a", gamepad1.a);
            telemetry.update();
        } else {
            box.setPower(gamepad1.left_stick_y/2);
            telemetry.addData("ninja", ninja);
            telemetry.addData("speed", gamepad1.left_stick_y/2);
            telemetry.addData("n", n);
            telemetry.addData("a", gamepad1.a);
            telemetry.update();

        }

        joint.setPower(gamepad1.right_stick_y/4);
    }
}
