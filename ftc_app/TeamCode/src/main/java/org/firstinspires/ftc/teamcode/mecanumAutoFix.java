package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class mecanumAutoFix extends LinearOpMode {
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;

    public Orientation angles;
    public BNO055IMU imu;

    public double currReturn = 0;
    public void runOpMode() throws InterruptedException{
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        double start = 0.5;
        double end = 2.0;

        for(int i=0; i<5;i++) {
            double currKey = (start + end) / 2;
            runMecanumCode(currKey);
            if(currReturn >= 2){
                start = currKey;
            } else if(currReturn <= -2){
                end = currKey;
            } else {
                break;
            }
        }

    }

    public void runMecanumCode(double k) throws InterruptedException{
        double first = getHeading();
        mecanumDrive(1,0,0,k);
        Thread.sleep(1000);
        double second = getHeading();
        mecanumDrive(-1,0,0,k);
        Thread.sleep(1000);
        double third = getHeading();
        currReturn =  ((second - first + second - third)/2);
    }
    public void mecanumDrive(double lx,double ly,double rx,double k){
        fl.setPower(Range.clip((1+k)*(ly + rx - lx),-1,1));
        bl.setPower(Range.clip((1-k)*(ly + rx + lx),-1,1));
        fr.setPower(Range.clip((1+k)*(ly - rx +lx),-1,1));
        br.setPower(Range.clip((1-k)*(ly - rx - lx),-1,1));

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


    //insert methods
}
