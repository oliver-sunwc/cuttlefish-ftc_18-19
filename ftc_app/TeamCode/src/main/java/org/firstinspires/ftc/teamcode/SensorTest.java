package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@TeleOp(name = "SensorTest")

public class SensorTest extends OpMode {
    /*NormalizedColorSensor sT;
    DistanceSensor sD;*/
    ColorSensor cs;
    DistanceSensor ds;
    //DistanceSensor revD;
    //AnalogInput ods;
    ColorSensor csv;
    DistanceSensor dsv;

    @Override
    public void init(){
        /*sT = hardwareMap.get(NormalizedColorSensor.class,"cs");
        sD = hardwareMap.get(DistanceSensor.class, "ds");*/
        //revD = hardwareMap.get(DistanceSensor.class, "revD");
        //ods  = hardwareMap.get(AnalogInput.class   ,  "ods");
        cs   = hardwareMap.get(ColorSensor.class   , "cs");
        ds   = hardwareMap.get(DistanceSensor.class, "ds");
        csv  = hardwareMap.get(ColorSensor.class, "csv");
        dsv  = hardwareMap.get(DistanceSensor.class, "dsv");
    }

    @Override
    public void loop(){
        /*NormalizedRGBA colors = sT.getNormalizedColors();
        int color = colors.toColor();
        telemetry.addLine("raw Android color: ")
                .addData("a", "%02x", Color.alpha(color))
                .addData("r", "%02x", Color.red(color))
                .addData("g", "%02x", Color.green(color))
                .addData("b", "%02x", Color.blue(color));
        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red   /= max;
        colors.green /= max;
        colors.blue  /= max;
        color = colors.toColor();

        telemetry.addLine("normalized color:  ")
                .addData("a", "%02x", Color.alpha(color))
                .addData("r", "%02x", Color.red(color))
                .addData("g", "%02x", Color.green(color))
                .addData("b", "%02x", Color.blue(color));
        telemetry.update();
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", sD.getDistance(DistanceUnit.CM)));
        telemetry.update();*/
        /*telemetry.addData("deviceName",revD.getDeviceName() );
        telemetry.addData("range", String.format("%.01f mm", revD.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", revD.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f m", revD.getDistance(DistanceUnit.METER)));
        telemetry.addData("range", String.format("%.01f in", revD.getDistance(DistanceUnit.INCH)));*/
        telemetry.addData("Red", cs.red());
        telemetry.addData("Green", cs.green());
        telemetry.addData("Blue", cs.blue());
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", ds.getDistance(DistanceUnit.CM)));
        telemetry.addData("Red V2", csv.red());
        telemetry.addData("Green V2", csv.green());
        telemetry.addData("Blue V2", csv.blue());
        telemetry.addData("Distance V2 (cm)",
                String.format(Locale.US, "%.02f", dsv.getDistance(DistanceUnit.CM)));
        telemetry.update();
    }
}
