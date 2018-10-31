package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, blue.
 *
 * Additionally, the centers of the bounding rectangles of the contours are sent to telemetry.
 */
@Autonomous(name="CurrentTest")
public class TestVisionOpMode extends LinearOpMode {
    private ExampleBlueVision blueVision;
    @Override

    public void runOpMode() throws InterruptedException{
        blueVision = new ExampleBlueVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        blueVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        blueVision.setShowCountours(true);
        // start the vision system
        blueVision.enable();
        waitForStart();


        blueVision.setShowCountours(true);

        // get a list of contours from the vision system
        List<MatOfPoint> contours = blueVision.getContours();
        boolean left;
        boolean right;
        boolean center;

        Thread.sleep(4000);
        contours = blueVision.getContours();
        sleep(500);
        telemetry.addData("performing","left view");
        telemetry.update();
        Thread.sleep(500);
        if(contours.size() > 0){
            left = true;
        } else {
            left = false;
        }


        Thread.sleep(4000);
        contours = blueVision.getContours();
        sleep(500);
        telemetry.addData("performing","center view");
        telemetry.update();
        Thread.sleep(500);
        if(contours.size() > 0){
            //get the lowest contour
            //check to the line
            center = true;
        } else {
            center = false;
        }

        Thread.sleep(4000);
        contours = blueVision.getContours();
        sleep(500);
        telemetry.addData("performing","right view");
        telemetry.update();
        Thread.sleep(500);
        if(contours.size() > 0){
            right = true;
        } else {
            right = false;
        }

        telemetry.addData("left",left);
        telemetry.addData("right",right);
        telemetry.addData("center",center);
        telemetry.update();
        /*for (int i = 0; i < contours.size(); i++) {
            // get the bounding rectangle of a single contour, we use it to get the x/y center
            // yes there's a mass center using Imgproc.moments but w/e

            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
            //double rgb[] = blueVision.hsv.get(0,0);

            telemetry.addData("contour" + Integer.toString(i),
                    String.format(Locale.getDefault(), "(%d, %d)", (boundingRect.x + boundingRect.width) / 2, (boundingRect.y + boundingRect.height) / 2));
            telemetry.addData("hi",contours.get(i));
            telemetry.addData("hi",contours.get(i).get(0,0));
        }*/

        //telemetry.update();
        sleep(10000000);


        blueVision.disable();
    }




}
