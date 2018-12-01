package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "odoTest")
public class odometerTest extends OpMode {
    DcMotor motor;
    ElapsedTime odo = new ElapsedTime();
    String time = "";

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "m");
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_y != 0) {
            motor.setPower(1);
            time = odo.time(TimeUnit.MILLISECONDS) + "";
        } else {
            if(odo.time(TimeUnit.MILLISECONDS) > 250) {
                try {
                    odoWrite(time);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            odo.reset();
            time = odo.time(TimeUnit.MILLISECONDS) + "";
        }
    }

    public static void odoWrite(String input) throws IOException{
        FileWriter fileWriter = new FileWriter("/sdcard/FIRST/odometer.txt");
        PrintWriter printWriter = new PrintWriter(fileWriter);
        printWriter.print(input);
    }
}


