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

@Autonomous(name="autCraterProfile")
public class autoCraterSideProfile extends LinearOpMode {
    roverHMAP robot;
    roverAuto robotAuto;
    private VisionThing vision;

    public void runOpMode() throws InterruptedException {
        robot = new roverHMAP();
        robot.init(hardwareMap,true);
        robotAuto = new roverAuto(robot);

        robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.flap.setPosition(0.5);

        //region setUp vision

        vision = new VisionThing();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();
        vision.setShowCountours(true);

        //endregion setUp vision

        //region init move line loop
        boolean testa = false;
        boolean testb = false;
        while(true){
            if(gamepad1.a && !testa){
                vision.pixelNum += 5;
            }
            testa = gamepad1.a;


            if(gamepad1.b && !testb){
                vision.pixelNum-= 5;
            }
            testb = gamepad1.b;


            if(isStarted()){
                break;
            }
            telemetry.addData("vision",vision.pixelNum);
            telemetry.update();
        }

        //endregion

        waitForStart();

        //region set dump arm up
        robot.flipLArm.setPosition(0.35);
        robot.flipRArm.setPosition(0.65);
        //endregion

        //region vision code
        int left=0;
        int right=0;
        int middle=0;
        for(int j=0;j<10;j++) {
            Thread.sleep(10);
            List<MatOfPoint> contours;
            contours = vision.getContours();
            int leftcounter = 0;
            int rightcounter = 0;
            String sees;
            if(contours.size() > 0) {
                for (int i = 0; i < contours.size(); i++) {
                    Rect boundRec = Imgproc.boundingRect(contours.get(i));
                    if (boundRec.y + boundRec.height > vision.pixelNum) {
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


        //endregion

        //region hang drop
        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currPos = robot.hang.getCurrentPosition();

        robot.hang.setPower(0.8);
        telemetry.addData("verdict:",verdict);
        telemetry.addData("gyro",getHeading());
        telemetry.addData("position:","brake disengaged");
        telemetry.update();
        Thread.sleep(500);
        robot.hang.setPower(0);

        robot.hang.setPower(-1);
        telemetry.addData("position:","firstDrop");
        telemetry.update();
        Thread.sleep(1550);
        robot.hang.setPower(0);



        robot.hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hang.setPower(0);
        Thread.sleep(10);
        //endregion

        vision.disable();


        //region attempt at slow down loop
        /*double power = 0.7;
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double distance = 900;
        int flDist = robot.fl.getCurrentPosition();
        int frDist = robot.fr.getCurrentPosition();
        int blDist = robot.bl.getCurrentPosition();
        int brDist = robot.br.getCurrentPosition();
        robotAuto.verticalDrive(power);
        ElapsedTime hi2 = new ElapsedTime();
        hi2.startTime();
        double prevms = hi2.milliseconds();
        int counter = 0;
            while (robot.fl.getCurrentPosition() - flDist < distance &&
                    robot.fr.getCurrentPosition() - frDist< distance &&
                    robot.bl.getCurrentPosition() - blDist< distance &&
                    robot.br.getCurrentPosition() - brDist< distance) {
                int hi = robot.fl.getCurrentPosition() - flDist;
                hi += robot.fr.getCurrentPosition() - frDist;
                hi += robot.bl.getCurrentPosition() - blDist;
                hi += robot.br.getCurrentPosition() - brDist;
                hi /= 4;
                if(hi < 400){
                    robotAuto.verticalDrive(power);
                }
                if(hi > 400){
                    int x = hi-400;
                    double powerThing = (power-0.2)*(500-x)/500;
                    robotAuto.verticalDrive(powerThing);
                    telemetry.addData("hi",hi);
                    telemetry.addData("hi2",hi2.milliseconds() - prevms);
                    prevms = hi2.milliseconds();
                }
            }

            robotAuto.moveForward(-0.05);
            Thread.sleep(50);
            robotAuto.moveForward(0);*/
        //endregion

        // region moveforward ten ticks and then go to 1cm on the distance sensor
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double distance = 10;
        double power = 0.1;
        int flDist = robot.fl.getCurrentPosition();
        int frDist = robot.fr.getCurrentPosition();
        int blDist = robot.bl.getCurrentPosition();
        int brDist = robot.br.getCurrentPosition();
        robotAuto.verticalDrive(power);
        ElapsedTime hi2 = new ElapsedTime();
        hi2.startTime();


        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotAuto.verticalDrive(0.1);
        while(robot.dist.getDistance(DistanceUnit.CM)< 1){
            telemetry.addData("dist",robot.dist.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        //endregion

        // region first arc loop toward depot
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.fl.setPower(0.8);
            robot.bl.setPower(0.8);
            robot.fr.setPower(-0.07);
            robot.br.setPower(-0.07);

            while(getHeading() < 45){
                telemetry.addData("hi",getHeading());
                telemetry.update();
            }


            while(getHeading() < 80){
                double hi = getHeading() - 45;
                double diff = 40-hi; //  gH = 85 - diff
                robot.fl.setPower(0.7 * diff/40 +0.1);
                robot.bl.setPower(0.7 * diff/40 +0.1);
                robot.fr.setPower(-0.25*(diff-30)/40);
                robot.br.setPower(-0.25*(diff-30)/40);

                telemetry.addData("hi",getHeading());
                telemetry.update();
            }

            robotAuto.stopDriving();
            Thread.sleep(50);
            //Thread.sleep(10000);
        //endregion

        //region go straight for a little bit, second arc loop toward depot and start extending slide
        robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         distance = 1430;
         power = 0.6;
         flDist = robot.fl.getCurrentPosition();
         frDist = robot.fr.getCurrentPosition();
         blDist = robot.bl.getCurrentPosition();
         brDist = robot.br.getCurrentPosition();
        robotAuto.verticalDrive(power);
        hi2.startTime();

        while (robot.fl.getCurrentPosition() - flDist < distance &&
                robot.fr.getCurrentPosition() - frDist< distance &&
                robot.bl.getCurrentPosition() - blDist< distance &&
                robot.br.getCurrentPosition() - brDist< distance) {
        }

        robot.fr.setPower(-0.07);
        robot.br.setPower(-0.07);
        while(getHeading() < 115){
            if(getHeading() > 100){
                robot.spine.setPower(-1);
            }
        }

        robotAuto.stopDriving();

        Thread.sleep(250);
        //endregion

        //region set dump arm and hang down and rotateArm flat
        robot.flipLArm.setPosition(1);
        robot.flipRArm.setPosition(0);
        robot.rotateArm.setPosition(0.5);

        robot.hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hang.setPower(0.5);
        //endregion

        //region set intake flip down, stop hang and spine, start intake
        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.inFlip.setPower(0.4);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 410);
        Thread.sleep(500);

        robot.inFlip.setPower(0);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        robot.hang.setPower(0);
        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hang.setPower(-0.1);
        robot.intake.setPower(-1);
        Thread.sleep(250);
        robot.hang.setPower(0);


        Thread.sleep(250);
        robot.spine.setPower(0);
        Thread.sleep(250);
        //endregion

        //region bring spine back and intake up
        robot.spine.setPower(1);
        Thread.sleep(250);
        robot.intake.setPower(0);

        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.inFlip.setPower(-0.8);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition()-440);
        Thread.sleep(500);

        robot.spine.setPower(0);
        //endregion

        //region arc loop back to depot
        robot.fl.setPower(-0.6);
        robot.bl.setPower(-0.6);
        robot.fr.setPower(0.35);
        robot.br.setPower(0.35);

        while(getHeading() > 100){
            if(getHeading() < 115){
                double diff = 115-getHeading();
                diff = 15-diff;
                robot.fl.setPower(-0.4*diff/15-0.2);
                robot.bl.setPower(-0.4*diff/15-0.2);
                robot.fr.setPower(0.2*diff/15+0.1);
                robot.br.setPower(0.2*diff/15+0.1);
            }
        }

        robotAuto.stopDriving();
        Thread.sleep(50);
        //endregion



        if(verdict.equals("left")){
            /*
            //region left case code
            //region drive straight back loop with power slow down
            robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            distance = -2500;
            power = -0.6;
            flDist = robot.fl.getCurrentPosition();
            frDist = robot.fr.getCurrentPosition();
            blDist = robot.bl.getCurrentPosition();
            brDist = robot.br.getCurrentPosition();
            robotAuto.verticalDrive(power);
            hi2.startTime();

            while (robot.fl.getCurrentPosition() - flDist > distance &&
                    robot.fr.getCurrentPosition() - frDist> distance &&
                    robot.bl.getCurrentPosition() - blDist> distance &&
                    robot.br.getCurrentPosition() - brDist> distance) {

                int thing1 = robot.fl.getCurrentPosition() - flDist;
                if(thing1 > -1900){

                }

                if(thing1< -1900){
                    int x1 = 2500 + thing1;
                    robotAuto.verticalDrive((power+0.15)*x1/600 - 0.15);
                }

            }

            robotAuto.verticalDrive(0.05);
            Thread.sleep(50);
            robotAuto.stopDriving();

            //endregion

            //region turn to angle to knock ball off
            while(getHeading() > 55){
                telemetry.addData("gyro",getHeading());
                telemetry.update();
                robot.fl.setPower(-0.25);
                robot.bl.setPower(-0.25);
                robot.fr.setPower(0.25);
                robot.br.setPower(0.25);
            }
            robotAuto.stopDriving();
            //endregion

            //region drop intake and extend spine
            robot.inFlip.setPower(0.4);
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 410);
            Thread.sleep(500);
            robot.intake.setPower(0.65);
            robot.spine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.spine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.spine.setPower(-1);
            int initPos = robot.spine.getCurrentPosition();

            ElapsedTime timer1 = new ElapsedTime();
            timer1.startTime();
            while(robot.spine.getCurrentPosition()-initPos > -650 && timer1.seconds() < 2.0){
                telemetry.addData("thing",robot.intake.getCurrentPosition());
                telemetry.addData("encoder",robot.spine.getCurrentPosition());
                telemetry.update();
            }
            //endregion

            //region bring intake up to transfer and spine back
            Thread.sleep(250);
            robot.inFlip.setPower(-0.8);
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() - 440);
            robot.spine.setPower(1);
            Thread.sleep(300);
            robot.intake.setPower(0);
            Thread.sleep(200);
            robot.spine.setPower(0);
            robot.intake.setPower(0.4);
            //endregion

            //region turn to dump angle and stop transfer
            while(getHeading() > -2){
                telemetry.addData("gyro",getHeading());
                telemetry.update();
                robot.fl.setPower(-0.4);
                robot.bl.setPower(-0.4);
                robot.fr.setPower(0.4);
                robot.br.setPower(0.4);
            }

            robot.intake.setPower(0);
            //endregion

            //region extend spine drive back and flip up to dump
            robot.spine.setPower(-1);
            robot.flipLArm.setPosition(0);
            robot.flipRArm.setPosition(1);
            robot.rotateArm.setPosition(0);
            robotAuto.verticalDriveDistance(-0.4,-6);
            robot.spine.setPower(0);
            Thread.sleep(500);
            //endregion

            //region first dump in lander loop
            robot.spine.setPower(-1);
            Thread.sleep(250);

            robot.flipLArm.setPosition(1);
            robot.flipRArm.setPosition(0);
            robot.rotateArm.setPosition(0.5);


            robotAuto.verticalDriveDistance(0.4,6);

            robot.inFlip.setPower(0.4);
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 410);


            robot.intake.setPower(0.7);
            Thread.sleep(500);
            robot.spine.setPower(1);
            Thread.sleep(150);
            robot.spine.setPower(0);
            Thread.sleep(150);
            robot.spine.setPower(-1);
            Thread.sleep(500);
            robot.spine.setPower(0);
            robot.intake.setPower(0.8);
            Thread.sleep(1250);
            robot.intake.setPower(0);

            robot.spine.setPower(1);
            robot.inFlip.setPower(-0.8);
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition()-440);
            robotAuto.verticalDriveDistance(-0.4,-6);
            Thread.sleep(750);
            robot.spine.setPower(0);

            robot.intake.setPower(0.5);
            Thread.sleep(750);
            robot.intake.setPower(0);

            robot.spine.setPower(-1);
            Thread.sleep(250);

            robot.flipLArm.setPosition(0);
            robot.flipRArm.setPosition(1);
            robot.rotateArm.setPosition(0);
            Thread.sleep(250);
            robot.spine.setPower(0);
            Thread.sleep(500);

            //endregion

            //region second dump in lander loop
            robot.spine.setPower(-1);

            Thread.sleep(250);


            robot.flipLArm.setPosition(1);
            robot.flipRArm.setPosition(0);
            robot.rotateArm.setPosition(0.5);


            robotAuto.verticalDriveDistance(0.4,6);

            robot.inFlip.setPower(0.4);
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 410);


            robot.intake.setPower(0.7);
            Thread.sleep(500);
            robot.spine.setPower(1);
            Thread.sleep(150);
            robot.spine.setPower(0);
            Thread.sleep(150);
            robot.spine.setPower(-1);
            Thread.sleep(500);
            robot.spine.setPower(0);
            robot.intake.setPower(0.8);
            Thread.sleep(1250);
            robot.intake.setPower(0);


            robot.spine.setPower(1);
            robot.inFlip.setPower(-0.8);
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition()-440);
            robotAuto.verticalDriveDistance(-0.4,-6);
            Thread.sleep(750);
            robot.spine.setPower(0);

            robot.intake.setPower(0.5);
            Thread.sleep(750);
            robot.intake.setPower(0);

            robot.spine.setPower(-1);
            Thread.sleep(250);

            robot.flipLArm.setPosition(0);
            robot.flipRArm.setPosition(1);
            robot.rotateArm.setPosition(0);
            Thread.sleep(250);
            robot.spine.setPower(0);
            Thread.sleep(500);
            //endregion
            //endregion

           */
        } else if(verdict.equals("middle")){
            //region middle case code

            //region drive back with power slow down
            robot.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            distance = -2250;
            power = -0.6;
            flDist = robot.fl.getCurrentPosition();
            frDist = robot.fr.getCurrentPosition();
            blDist = robot.bl.getCurrentPosition();
            brDist = robot.br.getCurrentPosition();
            robotAuto.verticalDrive(power);
            hi2.startTime();

            while (robot.fl.getCurrentPosition() - flDist > distance &&
                    robot.fr.getCurrentPosition() - frDist> distance &&
                    robot.bl.getCurrentPosition() - blDist> distance &&
                    robot.br.getCurrentPosition() - brDist> distance) {

                int thing1 = robot.fl.getCurrentPosition() - flDist;
                if(thing1 > -1450){

                }

                if(thing1< -1450){
                    int x1 = 2250 + thing1;
                    robotAuto.verticalDrive((power+0.15)*x1/800 - 0.15);
                }

            }

            robotAuto.verticalDrive(0.05);
            Thread.sleep(50);
            robotAuto.stopDriving();

            //endregion

            //region turn to angle to knock cube off and bring spine back a little
            robot.spine.setPower(0.05);
            while(getHeading() > 12){
                telemetry.addData("gyro",getHeading());
                telemetry.update();
                robot.fl.setPower(-0.6);
                robot.bl.setPower(-0.6);
                robot.fr.setPower(0.2);
                robot.br.setPower(0.2);
            }
            robotAuto.stopDriving();
            robot.spine.setPower(0);

            //endregion

            //region drop intake and extend spine while moving back a little
            robotAuto.verticalDrive(-0.13);
            Thread.sleep(250);

            robot.spine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.inFlip.setPower(0.4);
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 410);
            Thread.sleep(250);
            robot.intake.setPower(0.45);
            robot.spine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.spine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.spine.setPower(-1);
            int initPos = robot.spine.getCurrentPosition();


            ElapsedTime timer1 = new ElapsedTime();
            timer1.startTime();
            while(robot.spine.getCurrentPosition()-initPos > -270 && timer1.seconds() < 1.0){
                telemetry.addData("thing",robot.intake.getCurrentPosition());
                telemetry.addData("encoder",robot.spine.getCurrentPosition());
                telemetry.update();

                if(robot.dist.getDistance(DistanceUnit.CM) < 25){
                    robotAuto.stopDriving();
                }
            }

            robot.spine.setPower(0);
            hi2.reset();
            while(robot.dist.getDistance(DistanceUnit.CM) > 25){

            }

            robotAuto.stopDriving();

            robot.fl.setPower(-0.25);
            robot.bl.setPower(-0.25);
            robot.fr.setPower(0.25);
            robot.br.setPower(0.25);
            Thread.sleep(200);

            robot.fl.setPower(0.25);
            robot.bl.setPower(0.25);
            robot.fr.setPower(-0.25);
            robot.br.setPower(-0.25);
            Thread.sleep(400);

            robot.fl.setPower(-0.25);
            robot.bl.setPower(-0.25);
            robot.fr.setPower(0.25);
            robot.br.setPower(0.25);
            Thread.sleep(200);

            robotAuto.stopDriving();

            //endregion

            //region bring intake up to transfer and spine back
            Thread.sleep(250);
            robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.inFlip.setPower(-0.8);
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() - 440);
            robot.spine.setPower(1);
            Thread.sleep(300);
            robot.intake.setPower(0);
            Thread.sleep(200);
            robot.spine.setPower(0);
            robot.intake.setPower(0.4);
            //endregion

            //region turn to dump angle and stop transfer
            while(getHeading() > -7){
                telemetry.addData("gyro",getHeading());
                telemetry.update();
                robot.fl.setPower(-0.15);
                robot.bl.setPower(-0.15);
                robot.fr.setPower(0.15);
                robot.br.setPower(0.15);
            }

            robotAuto.stopDriving();
            Thread.sleep(250);

            robotAuto.verticalDrive(-0.4);
            while(robot.dist.getDistance(DistanceUnit.CM) > 8){
                if(robot.dist.getDistance(DistanceUnit.CM) > 20){
                }
            }

            robotAuto.verticalDrive(0.05);
            Thread.sleep(50);
            robotAuto.stopDriving();
            robot.intake.setPower(0);
            //endregion

            //region extend spine and flip up to dump
            robot.spine.setPower(-1);
            robot.flipLArm.setPosition(0);
            robot.flipRArm.setPosition(1);
            robot.rotateArm.setPosition(0.1);
            Thread.sleep(500);
            robot.spine.setPower(0);
            Thread.sleep(500);
            //endregion

            //region first dump in lander loop

            robot.spine.setPower(-1);

            robot.flipLArm.setPosition(1);
            robot.flipRArm.setPosition(0);
            robot.rotateArm.setPosition(0.5);


            robotAuto.verticalDrive(0.4);
            Thread.sleep(200);
            robot.spine.setPower(0);

            boolean bull = true;
            while(robot.dist.getDistance(DistanceUnit.CM) < 22){
                if(bull && robot.dist.getDistance(DistanceUnit.CM) > 18){
                    robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.inFlip.setPower(0.4);
                    robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 410);
                }
            }
            robotAuto.stopDriving();




            robot.intake.setPower(0.5);


            robot.inFlip.setPower(0);
            robot.inFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            robot.spine.setPower(1);
            Thread.sleep(300);
            robot.spine.setPower(0);
            Thread.sleep(250);
            robot.spine.setPower(-1);
            Thread.sleep(500);
            robot.spine.setPower(0);
            robot.intake.setPower(0.75);
            Thread.sleep(650);
            robot.intake.setPower(0);

            robot.intake.setPower(-0.3);
            Thread.sleep(200);

            robot.spine.setPower(1);

            robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.inFlip.setPower(-0.8);
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition()-440);

            robotAuto.verticalDrive(-0.4);
            while(robot.dist.getDistance(DistanceUnit.CM) > 15){
                if(robot.dist.getDistance(DistanceUnit.CM) > 20){
                }
            }
            robotAuto.verticalDrive(-0.05);
            Thread.sleep(50);
            robotAuto.stopDriving();


            Thread.sleep(50);
            robot.spine.setPower(0);

            robot.intake.setPower(0.5);
            Thread.sleep(700);
            robot.intake.setPower(0);



            robot.spine.setPower(-1);
            Thread.sleep(300);

            robot.flipLArm.setPosition(0);
            robot.flipRArm.setPosition(1);
            robot.rotateArm.setPosition(0.1);
            Thread.sleep(250);
            robot.spine.setPower(0);
            Thread.sleep(500);

            //endregion

            //region second dump in lander loop
            robot.spine.setPower(-1);

            Thread.sleep(250);


            robot.flipLArm.setPosition(1);
            robot.flipRArm.setPosition(0);
            robot.rotateArm.setPosition(0.5);


            robotAuto.verticalDrive(0.4);
            while(robot.dist.getDistance(DistanceUnit.CM) < 22){

            }
            robotAuto.stopDriving();

            robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.inFlip.setPower(0.4);
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 410);


            robot.intake.setPower(0.5);
            Thread.sleep(500);

            robot.inFlip.setPower(0);
            robot.inFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            robot.spine.setPower(1);
            Thread.sleep(150);
            robot.spine.setPower(0);
            Thread.sleep(150);
            robot.spine.setPower(-1);
            Thread.sleep(500);
            robot.spine.setPower(0);
            robot.intake.setPower(0.7);

            robot.fl.setPower(-0.25);
            robot.bl.setPower(-0.25);
            robot.fr.setPower(0.25);
            robot.br.setPower(0.25);
            Thread.sleep(325);

            robot.fl.setPower(0.25);
            robot.bl.setPower(0.25);
            robot.fr.setPower(-0.25);
            robot.br.setPower(-0.25);
            Thread.sleep(325);

            robotAuto.stopDriving();

            robot.intake.setPower(-0.3);
            Thread.sleep(250);

            robot.intake.setPower(0);


            robot.spine.setPower(1);

            robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.inFlip.setPower(-0.8);
            robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition()-440);

            robotAuto.verticalDrive(-0.4);
            while(robot.dist.getDistance(DistanceUnit.CM) > 15){
                if(robot.dist.getDistance(DistanceUnit.CM) > 20){
                }
            }

            robotAuto.verticalDrive(0.05);
            Thread.sleep(50);
            robotAuto.stopDriving();

            Thread.sleep(50);
            robot.spine.setPower(0);

            robot.intake.setPower(0.5);
            Thread.sleep(700);
            robot.intake.setPower(0);

            robot.spine.setPower(-1);
            Thread.sleep(300);

            robot.flipLArm.setPosition(0);
            robot.flipRArm.setPosition(1);
            robot.rotateArm.setPosition(0.1);
            Thread.sleep(250);
            Thread.sleep(500);
            //endregion



            //endregion
        } else{
            //region right code
            while(getHeading() > -30){
                telemetry.addData("gyro",getHeading());
                telemetry.update();
                robot.fl.setPower(-0.4);
                robot.bl.setPower(-0.4);
                robot.fr.setPower(0.4);
                robot.br.setPower(0.4);
            }
            robotAuto.stopDriving();
            //endregion
        }
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

