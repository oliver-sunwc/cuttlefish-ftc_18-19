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
            Thread.sleep(30);
            List<MatOfPoint> contours;
            contours = vision.getContours();
            int leftcounter = 0;
            int rightcounter = 0;
            String sees;
            if(contours.size() > 0) {
                for (int i = 0; i < contours.size(); i++) {
                    Rect boundRec = Imgproc.boundingRect(contours.get(i));
                    if (boundRec.y + boundRec.height > 4 * vision.givehsv().height() / 5 ) {
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


        double currAng = getHeading();

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
        Thread.sleep(350);
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

        /*if(currAng > getHeading()) {
            while(currAng > getHeading()) {
                robot.fl.setPower(0.1);
                robot.bl.setPower(0.1);
                robot.fr.setPower(-0.1);
                robot.br.setPower(-0.1);
            }
        } if(currAng < getHeading()) {
            while(currAng < getHeading()) {
                robot.fl.setPower(-0.1);
                robot.bl.setPower(-0.1);
                robot.fr.setPower(0.1);
                robot.br.setPower(0.1);
            }
        } else {

        }
        robotAuto.stopDriving();*/

        //robot.hang.setTargetPosition(robot.hang.getCurrentPosition());
        //robot.hang.setPower(0);

        Thread.sleep(250);
        telemetry.addData("dist",robot.dist.getDistance(DistanceUnit.CM));
        telemetry.update();
        vision.disable();

        //robotAuto.verticalDriveDistance(0.1,2);

        robotAuto.moveForward(0.1);
        while(robot.dist.getDistance(DistanceUnit.CM) < 10.5){
            telemetry.addData("dist",robot.dist.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        robotAuto.stopDriving();

        Thread.sleep(250);
         // gyro align





        robotAuto.stopDriving();
        if(verdict.equals("left")){
            if(getHeading() + 27 > 175){

                double curr = normalize(getHeading());
                while(normalize(getHeading()) < curr + 27){
                    robot.fl.setPower(0.2);
                    robot.bl.setPower(0.2);
                    robot.fr.setPower(-0.2);
                    robot.br.setPower(-0.2);
                }
                robotAuto.stopDriving();
            } else {
                double curr = getHeading();
                while(getHeading() < curr + 27){
                    robot.fl.setPower(0.2);
                    robot.bl.setPower(0.2);
                    robot.fr.setPower(-0.2);
                    robot.br.setPower(-0.2);
                }
                robotAuto.stopDriving();
            }
        } else if(verdict.equals("right")) {
            if(getHeading() - 27 < -175) {
                double curr = normalize(getHeading());
                while(normalize(getHeading()) > curr - 27){
                    robot.fl.setPower(-0.2);
                    robot.bl.setPower(-0.2);
                    robot.fr.setPower(0.2);
                    robot.br.setPower(0.2);
                }
                robotAuto.stopDriving();

            } else {
                double curr = getHeading();
                while(getHeading() > curr - 27){
                    robot.fl.setPower(-0.2);
                    robot.bl.setPower(-0.2);
                    robot.fr.setPower(0.2);
                    robot.br.setPower(0.2);
                }
                robotAuto.stopDriving();
            }
        } else {

        }

        telemetry.addData("gyro",getHeading());
        telemetry.update();
        robot.inFlip.setPower(0.4);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 470);

        Thread.sleep(500);

        robot.spine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.spine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.spine.setPower(-1);
        int curr1 = robot.spine.getCurrentPosition();
        ElapsedTime timerThing = new ElapsedTime();
        timerThing.startTime();
        int stupThing = 850;
        if(verdict.equals("middle")){
            stupThing = 550;
        }
        while(robot.spine.getCurrentPosition() > curr1 - stupThing && timerThing.seconds() < 2.8){
            telemetry.addData("thing",robot.intake.getCurrentPosition());
            telemetry.addData("encoder",robot.spine.getCurrentPosition());
            telemetry.update();
        }
        Thread.sleep(100);


        robot.spine.setPower(0);
        telemetry.addData("encoder2",robot.spine.getCurrentPosition());
        telemetry.update();
        Thread.sleep(250);

        timerThing.reset();
        robot.spine.setPower(0.7);
        while(robot.spine.getCurrentPosition() < curr1 - 50 && timerThing.seconds() < 1.6){
            telemetry.addData("encoder",robot.spine.getCurrentPosition());
            telemetry.update();
        }

        robot.spine.setPower(0);
        Thread.sleep(100);

        robot.inFlip.setPower(-0.8);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() - 510);

        if(verdict.equals("left")) {
            if(getHeading() - 27 < -175) {
                double curr = normalize(getHeading());
                while(normalize(getHeading()) > curr - 27){
                    robot.fl.setPower(-0.2);
                    robot.bl.setPower(-0.2);
                    robot.fr.setPower(0.2);
                    robot.br.setPower(0.2);
                }
                robotAuto.stopDriving();

            } else {
                double curr = getHeading();
                while(getHeading() > curr - 27){
                    robot.fl.setPower(-0.2);
                    robot.bl.setPower(-0.2);
                    robot.fr.setPower(0.2);
                    robot.br.setPower(0.2);
                }
                robotAuto.stopDriving();
            }
        }

        if(verdict.equals("right")){
            if(getHeading() + 27 > 175){

                double curr = normalize(getHeading());
                while(normalize(getHeading()) < curr + 27){
                    robot.fl.setPower(0.2);
                    robot.bl.setPower(0.2);
                    robot.fr.setPower(-0.2);
                    robot.br.setPower(-0.2);
                }
                robotAuto.stopDriving();
            } else {
                double curr = getHeading();
                while(getHeading() < curr + 27){
                    robot.fl.setPower(0.2);
                    robot.bl.setPower(0.2);
                    robot.fr.setPower(-0.2);
                    robot.br.setPower(-0.2);
                }
                robotAuto.stopDriving();
            }
        }

        Thread.sleep(150);
        robotAuto.verticalDriveDistance(0.1,6.7);

        Thread.sleep(150);

        if(getHeading() + 85 > 175){

            double curr = normalize(getHeading());
            while(normalize(getHeading()) < curr + 80){
                robot.fl.setPower(0.2);
                robot.bl.setPower(0.2);
                robot.fr.setPower(-0.2);
                robot.br.setPower(-0.2);
            }
            robotAuto.stopDriving();
        } else {
            double curr = getHeading();
            while(getHeading() < curr + 80){
                robot.fl.setPower(0.2);
                robot.bl.setPower(0.2);
                robot.fr.setPower(-0.2);
                robot.br.setPower(-0.2);
            }
            robotAuto.stopDriving();
        }

        Thread.sleep(150);
        robotAuto.verticalDriveDistance(0.6,22);
        robotAuto.stopDriving();
        Thread.sleep(400);
        if(getHeading() + 23 > 175){

            double curr = normalize(getHeading());
            while(normalize(getHeading()) < curr + 23){
                robot.fl.setPower(0.2);
                robot.bl.setPower(0.2);
                robot.fr.setPower(-0.2);
                robot.br.setPower(-0.2);
            }
            robotAuto.stopDriving();
        } else {
            double curr = getHeading();
            while(getHeading() < curr + 23){
                robot.fl.setPower(0.2);
                robot.bl.setPower(0.2);
                robot.fr.setPower(-0.2);
                robot.br.setPower(-0.2);
            }
            robotAuto.stopDriving();
        }

        Thread.sleep(200);
        int curr2 = robot.spine.getCurrentPosition();
        robot.spine.setPower(-1);
        Thread.sleep(500);

        robot.inFlip.setPower(0.4);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 470);
        timerThing.reset();
        stupThing = 1300;
        while(robot.spine.getCurrentPosition() > curr2 - stupThing && timerThing.seconds() < 1.7){
            telemetry.addData("curr2",curr2);
            telemetry.addData("thing",robot.intake.getCurrentPosition());
            telemetry.addData("encoder",robot.spine.getCurrentPosition());
            telemetry.update();
        }


        Thread.sleep(100);

        telemetry.addData("encoder1",robot.spine.getCurrentPosition());
        telemetry.addData("curr2",curr2);
        telemetry.update();
        robot.spine.setPower(0);
        robot.intake.setPower(1);
        Thread.sleep(1500);
        robot.intake.setPower(0);
        robot.spine.setPower(1);
        robot.inFlip.setPower(-0.4);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() - 510);
        Thread.sleep(1500);
        robot.spine.setPower(0);
        //rotate
        // extendo slido
        Thread.sleep(200);

        robotAuto.verticalDriveDistance(-0.2, -7);

        Thread.sleep(200);

        if(getHeading() - 143 < -175) {
            double curr = normalize(getHeading());
            while(normalize(getHeading()) > curr - 143){
                robot.fl.setPower(-0.2);
                robot.bl.setPower(-0.2);
                robot.fr.setPower(0.2);
                robot.br.setPower(0.2);
            }
            robotAuto.stopDriving();

        } else {
            double curr = getHeading();
            while(getHeading() > curr - 143){
                robot.fl.setPower(-0.2);
                robot.bl.setPower(-0.2);
                robot.fr.setPower(0.2);
                robot.br.setPower(0.2);
            }
            robotAuto.stopDriving();
        }


        robot.spine.setPower(-1);
        Thread.sleep(2500);
        robot.spine.setPower(0);



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

