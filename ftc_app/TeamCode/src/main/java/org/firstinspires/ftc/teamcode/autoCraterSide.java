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
        robot.init(hardwareMap,true);
        robotAuto = new roverAuto(robot);
        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        vision = new VisionThing();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();
        vision.setShowCountours(true);
        waitForStart();

        telemetry.addData("gyro",getHeading());
        telemetry.update();

        robot.flipLArm.setPosition(0.35);
        robot.flipRArm.setPosition(0.65);

        sleep(250);
        int left=0;
        int right=0;
        int middle=0;
        for(int j=0;j<15;j++) {
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
        telemetry.addData("gyro",getHeading());
        telemetry.addData("position:","brake disengaged");
        telemetry.update();
        Thread.sleep(400);
        robot.hang.setPower(0);

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

        Thread.sleep(250);
        telemetry.addData("dist",robot.dist.getDistance(DistanceUnit.CM));
        telemetry.update();

        //robotAuto.verticalDriveDistance(0.1,2);

        robotAuto.moveForward(0.1);
        while(robot.dist.getDistance(DistanceUnit.CM) < 10){

        }
        robotAuto.stopDriving();


         // gyro align

        robot.inFlip.setPower(0.4);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 510);

        Thread.sleep(250);

        /*robotAuto.moveForward(0.2);

        while(robot.dist.getDistance(DistanceUnit.CM) < 30){
            telemetry.addData("thing","indicator");
            telemetry.addData("dist",robot.dist.getDistance(D istanceUnit.CM));
            telemetry.update();

        }*/

        robotAuto.stopDriving();
        if(verdict.equals("left")){
            if(getHeading() + 25 > 175){

                double curr = normalize(getHeading());
                while(normalize(getHeading()) < curr + 25){
                    robot.fl.setPower(0.1);
                    robot.bl.setPower(0.1);
                    robot.fr.setPower(-0.1);
                    robot.br.setPower(-0.1);
                }
                robotAuto.stopDriving();
            } else {
                double curr = getHeading();
                while(getHeading() < curr + 25){
                    robot.fl.setPower(0.1);
                    robot.bl.setPower(0.1);
                    robot.fr.setPower(-0.1);
                    robot.br.setPower(-0.1);
                }
                robotAuto.stopDriving();
            }
        } else if(verdict.equals("right")) {
            if(getHeading() - 25 < -175) {
                double curr = normalize(getHeading());
                while(normalize(getHeading()) > curr - 25){
                    robot.fl.setPower(-0.1);
                    robot.bl.setPower(-0.1);
                    robot.fr.setPower(0.1);
                    robot.br.setPower(0.1);
                }
                robotAuto.stopDriving();

            } else {
                double curr = getHeading();
                while(getHeading() > curr - 25){
                    robot.fl.setPower(-0.1);
                    robot.bl.setPower(-0.1);
                    robot.fr.setPower(0.1);
                    robot.br.setPower(0.1);
                }
                robotAuto.stopDriving();
            }
        } else {

        }

        telemetry.addData("gyro",getHeading());
        telemetry.update();

        robot.spine.setPower(-1);
        Thread.sleep(1500);

        robot.spine.setPower(0);
        Thread.sleep(250);

        robot.spine.setPower(1);
        Thread.sleep(1250);

        robot.spine.setPower(0);
        Thread.sleep(100);

        robot.inFlip.setPower(-0.8);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() - 510);

        if(verdict.equals("left")) {
            if(getHeading() - 22 < -175) {
                double curr = normalize(getHeading());
                while(normalize(getHeading()) > curr - 22){
                    robot.fl.setPower(-0.2);
                    robot.bl.setPower(-0.2);
                    robot.fr.setPower(0.2);
                    robot.br.setPower(0.2);
                }
                robotAuto.stopDriving();

            } else {
                double curr = getHeading();
                while(getHeading() > curr - 22){
                    robot.fl.setPower(-0.2);
                    robot.bl.setPower(-0.2);
                    robot.fr.setPower(0.2);
                    robot.br.setPower(0.2);
                }
                robotAuto.stopDriving();
            }
        }

        if(verdict.equals("right")){
            if(getHeading() + 22 > 175){

                double curr = normalize(getHeading());
                while(normalize(getHeading()) < curr + 22){
                    robot.fl.setPower(0.2);
                    robot.bl.setPower(0.2);
                    robot.fr.setPower(-0.2);
                    robot.br.setPower(-0.2);
                }
                robotAuto.stopDriving();
            } else {
                double curr = getHeading();
                while(getHeading() < curr + 22){
                    robot.fl.setPower(0.2);
                    robot.bl.setPower(0.2);
                    robot.fr.setPower(-0.2);
                    robot.br.setPower(-0.2);
                }
                robotAuto.stopDriving();
            }
        }

        Thread.sleep(250);
        robotAuto.verticalDriveDistance(0.3,7.5);

        Thread.sleep(250);

        if(getHeading() + 85 > 175){

            double curr = normalize(getHeading());
            while(normalize(getHeading()) < curr + 85){
                robot.fl.setPower(0.2);
                robot.bl.setPower(0.2);
                robot.fr.setPower(-0.2);
                robot.br.setPower(-0.2);
            }
            robotAuto.stopDriving();
        } else {
            double curr = getHeading();
            while(getHeading() < curr + 85){
                robot.fl.setPower(0.2);
                robot.bl.setPower(0.2);
                robot.fr.setPower(-0.2);
                robot.br.setPower(-0.2);
            }
            robotAuto.stopDriving();
        }

        robotAuto.verticalDriveDistance(0.7,18);

        if(getHeading() + 20 > 175){

            double curr = normalize(getHeading());
            while(normalize(getHeading()) < curr + 20){
                robot.fl.setPower(0.2);
                robot.bl.setPower(0.2);
                robot.fr.setPower(-0.2);
                robot.br.setPower(-0.2);
            }
            robotAuto.stopDriving();
        } else {
            double curr = getHeading();
            while(getHeading() < curr + 20){
                robot.fl.setPower(0.2);
                robot.bl.setPower(0.2);
                robot.fr.setPower(-0.2);
                robot.br.setPower(-0.2);
            }
            robotAuto.stopDriving();
        }

        robotAuto.verticalDriveDistance(0.4,7);
        robot.spine.setPower(-1);
        Thread.sleep(1800);
        robot.spine.setPower(0);
        Thread.sleep(500);
        robot.spine.setPower(1);
        Thread.sleep(1500);
        robot.spine.setPower(0);

        //rotate
        // extendo slido

        vision.disable();


    }

    void checkStop(){
        if(isStopRequested()){
            vision.disable();
        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /*
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return*/

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public double getHeading() {
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Double.parseDouble(formatAngle(robot.angles.angleUnit, robot.angles.firstAngle));
    }

    public double normalize(double hi){
        if(hi < 0){
            hi = -hi;
            hi = 180-hi;
            hi += 180;
        }
        return hi;
    }
}

