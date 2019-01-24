package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name="autCrater")
public class autoCraterSide extends LinearOpMode {
    roverHMAP robot;
    roverAuto robotAuto;
    private VisionThing vision;

    public void runOpMode() throws InterruptedException {
        robot = new roverHMAP();
        robot.init(hardwareMap);
        robotAuto = new roverAuto(robot);

        vision = new VisionThing();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();
        vision.setShowCountours(true);
        waitForStart();

        robot.flipLArm.setPosition(0.35);
        robot.flipRArm.setPosition(0.65);

        int left=0;
        int right=0;
        int middle=0;
        for(int j=0;j<20;j++) {
            Thread.sleep(50);
            List<MatOfPoint> contours;
            contours = vision.getContours();
            int leftcounter = 0;
            int rightcounter = 0;
            String sees;
            if(contours.size() > 0) {
                for (int i = 0; i < contours.size(); i++) {
                    Rect boundRec = Imgproc.boundingRect(contours.get(i));
                    if (boundRec.y + boundRec.height > 3 * vision.givehsv().height() / 4) {
                        if (boundRec.x + boundRec.width / 2 > vision.givehsv().width() / 2) {
                            rightcounter++;
                        } else {
                            leftcounter++;
                        }
                    }
                }
                if (leftcounter > 0 || rightcounter > 0) {
                    if (leftcounter > rightcounter) {
                        sees = "left";
                        left++;
                    } else if (rightcounter > leftcounter) {
                        sees = "middle";
                        middle++;
                    } else {
                        sees = "right";
                        right++;
                    }
                } else {
                    sees = "right";
                    right++;
                }
                telemetry.addData("sees mineral", sees);
                telemetry.addData("width", vision.givehsv().width());
                telemetry.addData("height", vision.givehsv().height());
                telemetry.addData("leftcounter", leftcounter);
                telemetry.addData("rightcounter", rightcounter);
                telemetry.update();
            }
            checkStop();
        }
        String verdict = "";
        if(left > right){
            if(left > middle){
                verdict = "left";
            } else {
                verdict = "middle";
            }
        } else {
            if(right > middle){
                verdict = "right";
            } else {
                verdict = "middle";
            }
        }




        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currPos = robot.hang.getCurrentPosition();
        //robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //robot.hang.setTargetPosition(robot.hang.getCurrentPosition() + 700);
        robot.hang.setPower(0.8);
        telemetry.addData("verdict:",verdict);
        telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.addData("position:","brake disengaged");
        telemetry.update();
        Thread.sleep(500);
        robot.hang.setPower(0);
        sleep(1000);

        //robot.hang.setTargetPosition(robot.hang.getCurrentPosition() - 7000);
        currPos = robot.hang.getCurrentPosition();
        robot.hang.setPower(-1);
        telemetry.addData("position:","firstDrop");
        telemetry.update();
        Thread.sleep(1500);
        robot.hang.setPower(0);

        /*robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hang.setTargetPosition(robot.hang.getCurrentPosition() - 7000);
        robot.hang.setPower(-0.9);*/
        telemetry.addData("position:","secondDrop");
        telemetry.update();

        //gyro align

        //robot.hang.setTargetPosition(robot.hang.getCurrentPosition());
        //robot.hang.setPower(0);

        Thread.sleep(1000);
        telemetry.addData("dist",robot.dist.getDistance(DistanceUnit.CM));
        telemetry.update();

        robotAuto.verticalDriveDistance(0.1,3);

         // gyro align
        Thread.sleep(1000);

        robot.inFlip.setPower(-0.9);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() - 110);
        /*robotAuto.moveForward(0.2);

        while(robot.dist.getDistance(DistanceUnit.CM) < 30){
            telemetry.addData("thing","indicator");
            telemetry.addData("dist",robot.dist.getDistance(D istanceUnit.CM));
            telemetry.update();

        }*/

        robotAuto.stopDriving();
        if(verdict.equals("left")){

        } else if(verdict.equals("right")) {

        } else {

        }

        robot.spine.setPower(0.6);
        Thread.sleep(1000);

        robot.spine.setPower(0);
        Thread.sleep(250);

        robot.spine.setPower(-0.6);
        Thread.sleep(1000);

        robot.spine.setPower(0);
        Thread.sleep(250);
        /*telemetry.addData("gyro",robotAuto.getHeading());
        telemetry.update();
        Thread.sleep(2000);

        // do vision thing
        while(robotAuto.getHeading() > -90){
            telemetry.addData("gyro",robotAuto.getHeading());
            telemetry.update();
            robot.fl.setPower(0.2);
            robot.bl.setPower(0.2);
            robot.br.setPower(-0.2);
            robot.fr.setPower(-0.2);
        }
        robotAuto.stopDriving();

        telemetry.addData("gyro", robotAuto.getHeading());
        telemetry.update();
        sleep(50000);

        //align with sensor
        /*robotAuto.verticalDrive(0.3);
        if(robot.landerS.getVoltage()*robot.voltage_to_in > 6) {
            robotAuto.stopDriving();
        }*/

        /*
        extend slide and drop intake loop
         */

        /*
        intake the cube
        */
        vision.disable();


    }

    void checkStop(){
        if(isStopRequested()){
            vision.disable();
        }
    }
}

