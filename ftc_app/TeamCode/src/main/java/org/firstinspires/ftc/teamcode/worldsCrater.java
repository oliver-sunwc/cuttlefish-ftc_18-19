package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

import java.util.List;
import java.util.Locale;

@Autonomous(name="worldsCrater")
public class worldsCrater extends LinearOpMode {
    roverHMAP robot;
    roverAuto robotAuto;
    private VisionThing vision;


    public void runOpMode() throws InterruptedException    {
        ElapsedTime timer1 = new ElapsedTime();
        timer1.startTime();
        robot = new roverHMAP();
        robot.init(hardwareMap,true);
        robotAuto = new roverAuto(robot);

        robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.dump.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,new PIDCoefficients(4,0,0));
        robot.dump.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dump.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.spine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.spine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.spine.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,new PIDCoefficients(10,2,0));
        robot.spine.setTargetPositionTolerance(15);
        double dumpFastPow = 0.7;
        double dumpSlowPow = 0.5;

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


        //region set dump arm
        robot.rotateArm.setPosition(0.1);
        robot.dump.setTargetPosition(-600);
        robot.dump.setPower(-0.8);
        while(robot.dump.getCurrentPosition() > -475){
            telemetry.addData("dump",robot.dump.getCurrentPosition());
        }
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
                    if(boundRec.width*boundRec.height > 10) {
                        if (boundRec.y + boundRec.height > vision.pixelNum) {
                            if (boundRec.x + boundRec.width / 2 > vision.givehsv().width() / 2) {
                                rightcounter++;
                            } else {
                                leftcounter++;
                            }
                        }
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
        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hang.setPower(1);
        while(robot.hang.getCurrentPosition() < 50) {
            telemetry.addData("hang",robot.hang.getCurrentPosition());
            telemetry.addData("verdict:", verdict);
            telemetry.addData("gyro", getHeading());
            telemetry.addData("position:", "brake disengaged");
            telemetry.addData("left", left);
            telemetry.addData("right", right);
            telemetry.update();
        }
        robot.hang.setPower(0);

        robot.hang.setPower(-1);
        while(robot.hang.getCurrentPosition() > -4500) {
            telemetry.addData("verdict:", verdict);
            telemetry.addData("hangpos", robot.hang.getCurrentPosition());
            telemetry.addData("hangpow", robot.hang.getPower());
            telemetry.update();
        }
        sleep(10);
        robot.hang.setPower(0);
        //endregion


        vision.disable();

        //region pid coefficients
        double p = 12;
        double i = 0.2;
        double d = 0.3;

        PIDCoefficients hi = new PIDCoefficients(p,i,d);

        robot.fl.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, hi);
        robot.fr.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, hi);
        robot.bl.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, hi);
        robot.br.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, hi);

        robotAuto.stopAndReset();

        robotAuto.runToPosition();
        //endregion

        //region first arc loop from lander
        robot.fl.setTargetPosition(3000);
        robot.fr.setTargetPosition(99);
        robot.bl.setTargetPosition(3000);
        robot.br.setTargetPosition(99);

        robot.fl.setPower(0.95);
        robot.bl.setPower(0.95);
        robot.fr.setPower(0.033);
        robot.br.setPower(0.033);

        robot.fl.setTargetPositionTolerance(10);
        robot.bl.setTargetPositionTolerance(10);
        robot.fr.setTargetPositionTolerance(10);
        robot.br.setTargetPositionTolerance(10);

        robot.inFlip.setPosition(0.15);
        robot.trapDoor.setPosition(1);
        //12 0.2 1
        //12 2 2
        //12 0.5 1.5

        robot.rotateArm.setPosition(0.65);
        robot.dumpFlip.setPosition(0.3);
        while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){


        }
        telemetry.addData("fl",robot.fl.getCurrentPosition());
        telemetry.addData("bl",robot.bl.getCurrentPosition());
        telemetry.addData("br",robot.br.getCurrentPosition());
        telemetry.addData("fr",robot.fr.getCurrentPosition());
        telemetry.addData("fl",robot.fl.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
        telemetry.update();

        robotAuto.verticalDrive(0);
        telemetry.addData("done","done");
        telemetry.update();
        //endregion

        //region second arc loop to depot + spine extension and retraction
        robotAuto.stopAndReset();
        robotAuto.runToPosition();

        robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hang.setTargetPosition(-2350);
        robot.hang.setPower(0.5);

        telemetry.addData("fl",robot.fl.getCurrentPosition());
        telemetry.addData("bl",robot.bl.getCurrentPosition());
        telemetry.addData("br",robot.br.getCurrentPosition());
        telemetry.addData("fr",robot.fr.getCurrentPosition());
        telemetry.addData("fl",robot.fl.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
        telemetry.update();

        robot.fl.setTargetPositionTolerance(30);
        robot.bl.setTargetPositionTolerance(30);
        robot.fr.setTargetPositionTolerance(30);
        robot.br.setTargetPositionTolerance(30);

        robot.fl.setTargetPosition(2700);//2700 & 1333
        robot.bl.setTargetPosition(2700);
        robot.fr.setTargetPosition(1333);
        robot.br.setTargetPosition(1333);

        robot.fl.setPower(0.9);
        robot.bl.setPower(0.9);
        robot.br.setPower(0.5);
        robot.fr.setPower(0.5);



        while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){
            if(robot.fl.getCurrentPosition() > 1500 || robot.bl.getCurrentPosition() > 1500){
                robot.spine.setPower(-1);
                robot.spine.setTargetPosition(-1200);
                robot.dump.setTargetPosition(500);
                robot.dump.setPower(0.2);
                robot.inFlip.setPosition(0.3);
            }
            telemetry.addData("spine",robot.spine.getCurrentPosition());
            telemetry.update();
        }


        Thread.sleep(100);

        robot.fl.setTargetPosition(robot.fl.getCurrentPosition() + 420-20);//2700 & 1333
        robot.bl.setTargetPosition(robot.bl.getCurrentPosition() + 420-20);
        robot.fr.setTargetPosition(robot.fr.getCurrentPosition() + 420-20);
        robot.br.setTargetPosition(robot.br.getCurrentPosition() + 420-20);

        robot.fl.setPower(0.35);
        robot.bl.setPower(0.35);
        robot.br.setPower(0.35);
        robot.fr.setPower(0.35);

        while(robot.spine.isBusy()){
            if(robot.spine.getCurrentPosition() < -850){
                robot.intake.setPower(1);
            }
        }

        if(robot.spine.getCurrentPosition() < -950){
            robot.intake.setPower(1);
        }

        Thread.sleep(50);
        while(robot.fl.isBusy() || robot.bl.isBusy() || robot.br.isBusy() || robot.fr.isBusy()){

        }
        Thread.sleep(50);
        robot.spine.setPower(1);
        robot.spine.setTargetPosition(-325);

        robot.fl.setTargetPositionTolerance(10);
        robot.bl.setTargetPositionTolerance(10);
        robot.fr.setTargetPositionTolerance(10);
        robot.br.setTargetPositionTolerance(10);
        //endregion

        sleep(300);
        robot.intake.setPower(0);
        robot.inFlip.setPosition(0.15);
        //region rotate away from depot
        robotAuto.runUsing();

        robot.fl.setPower(-0.5);
        robot.bl.setPower(-0.5);
        robot.fr.setPower(0.1);
        robot.br.setPower(0.1);
        while(robotAuto.getHeading() > 91){
            if(getHeading() < 110){
                robot.fl.setPower(-0.2);
                robot.bl.setPower(-0.2);
                robot.fr.setPower(0.04);
                robot.br.setPower(0.04);
            }
        }

        robotAuto.stopDriving();
        Thread.sleep(50);
        //endregion

        //region drive back
        robotAuto.stopAndReset();
        Thread.sleep(10);
        robotAuto.runToPosition();
        Thread.sleep(10);

        robot.fl.setTargetPosition(-2150);
        robot.bl.setTargetPosition(-2150);
        robot.fr.setTargetPosition(-2150);
        robot.br.setTargetPosition(-2150);

        robot.fl.setPower(-0.3);
        robot.bl.setPower(-0.3);
        robot.br.setPower(-0.3);
        robot.fr.setPower(-0.3);

        while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){
            if(robot.fl.getCurrentPosition() < -100){
                robot.fl.setPower(-0.9);
                robot.bl.setPower(-0.9);
                robot.fr.setPower(-0.9);
                robot.br.setPower(-0.9);
            }

        }

        robotAuto.stopDriving();
        Thread.sleep(50);
        //endregion

        //region case code
        if(verdict.equals("left")){
            //region left case code

            robotAuto.stopAndReset();
            robotAuto.runUsing();
            robot.inFlip.setPosition(0.65);

            robot.fl.setPower(-0.4);
            robot.bl.setPower(-0.4);
            robot.fr.setPower(0.4);
            robot.br.setPower(0.4);
            while(getHeading() > 38){
                if(getHeading() < 60){
                    robot.fl.setPower(-0.1);
                    robot.bl.setPower(-0.1);
                    robot.fr.setPower(0.1);
                    robot.br.setPower(0.1);
                }
            }

            robotAuto.stopDriving();

            robot.intake.setPower(-1);
            robot.spine.setPower(-1);
            robot.spine.setTargetPosition(-850);
            while(robot.spine.isBusy()){
                telemetry.addData("spinePos", robot.spine.getCurrentPosition());
                telemetry.addData("spinePow", robot.spine.getPower());
                telemetry.update();

                if(robot.spine.getCurrentPosition() < -800){
                    robot.inFlip.setPosition(0.15);
                }
            }
            robot.spine.setPower(1);
            robot.spine.setTargetPosition(-325);
            robot.trapDoor.setPosition(0.13);
            Thread.sleep(250);
            while(robot.spine.isBusy()){

            }

            robot.fl.setPower(-0.4);
            robot.bl.setPower(-0.4);
            robot.fr.setPower(0.4);
            robot.br.setPower(0.4);

            while(getHeading() > 3){
                if(getHeading() > 21){
                    robot.fl.setPower(-0.15);
                    robot.bl.setPower(-0.15);
                    robot.fr.setPower(0.15);
                    robot.br.setPower(0.15);
                }
            }
            robotAuto.stopDriving();
            robot.trapDoor.setPosition(1);
            Thread.sleep(100);
            robot.spine.setPower(-1);
            robot.spine.setTargetPosition(-850);

            robotAuto.stopAndReset();
            robotAuto.runToPosition();

            robot.fl.setTargetPosition(-950);
            robot.bl.setTargetPosition(-950);
            robot.fr.setTargetPosition(-950);
            robot.br.setTargetPosition(-950);

            robot.fl.setPower(-0.7);
            robot.bl.setPower(-0.7);
            robot.br.setPower(-0.7);
            robot.fr.setPower(-0.7);

            robot.dumpFlip.setPosition(0.1);
            timer1.reset();
            robot.dump.setTargetPosition(-800);
            robot.dump.setPower(-dumpFastPow);
            while(timer1.seconds() < 0.6){
                if(timer1.seconds() > 0.3){
                    robot.dumpFlip.setPosition(0.9);
                }
            }
            robot.rotateArm.setPosition(0.5);
            robot.dump.setPower(-dumpSlowPow);

            robotAuto.stopDriving();

            Thread.sleep(1000);
            //endregion
        } else if(verdict.equals("middle")){


            //region middle case code

            robotAuto.stopAndReset();
            robotAuto.runUsing();

            robot.fl.setPower(-0.4);
            robot.bl.setPower(-0.4);
            robot.fr.setPower(0.4);
            robot.br.setPower(0.4);
            while(getHeading() > -10){
                if(getHeading() < 22){
                    robot.fl.setPower(-0.15);
                    robot.bl.setPower(-0.15);
                    robot.fr.setPower(0.15);
                    robot.br.setPower(0.15);
                }

                if(getHeading()< 12){
                    robot.inFlip.setPosition(0.65);
                }
            }

            robotAuto.stopDriving();
            robot.inFlip.setPosition(0.65);
            robotAuto.verticalDriveDistance(-0.3,-2.5);
            robot.intake.setPower(-1);
            robot.spine.setPower(-1);
            robot.spine.setTargetPosition(-800);
            while(robot.spine.isBusy()){
                telemetry.addData("spinePos", robot.spine.getCurrentPosition());
                telemetry.addData("spinePow", robot.spine.getPower());
                telemetry.update();

                if(robot.spine.getCurrentPosition() < -750){
                    robot.inFlip.setPosition(0.15);
                }
            }
            robot.spine.setPower(1);
            robot.spine.setTargetPosition(-325);
            robot.trapDoor.setPosition(0.13);
            Thread.sleep(250);
            while(robot.spine.isBusy()){

            }


            while(getHeading() < -5){
                    robot.fl.setPower(0.15);
                    robot.bl.setPower(0.15);
                    robot.fr.setPower(-0.15);
                    robot.br.setPower(-0.15);
            }
            robotAuto.stopDriving();
            robot.trapDoor.setPosition(1);
            Thread.sleep(100);
            robot.spine.setPower(-1);
            robot.spine.setTargetPosition(-850);

            robotAuto.stopAndReset();
            robotAuto.runToPosition();

            robot.fl.setTargetPosition(-850);
            robot.bl.setTargetPosition(-850);
            robot.fr.setTargetPosition(-850);
            robot.br.setTargetPosition(-850);

            robot.fl.setPower(-0.7);
            robot.bl.setPower(-0.7);
            robot.br.setPower(-0.7);
            robot.fr.setPower(-0.7);

            robot.dumpFlip.setPosition(0.1);
            timer1.reset();
            robot.dump.setTargetPosition(-800);
            robot.dump.setPower(-dumpFastPow);
            while(timer1.seconds() < 0.6){
                if(timer1.seconds() > 0.3){
                    robot.dumpFlip.setPosition(0.9);
                }
            }
            robot.rotateArm.setPosition(0.5);
            robot.dump.setPower(-dumpSlowPow);

            robotAuto.stopDriving();

            Thread.sleep(1000);
            //endregion
            //region
        } else if(verdict.equals("right")){
            //region right case code
            /*
            robotAuto.stopAndReset();
            robotAuto.runUsing();
            robot.inFlip.setPosition(0.65);

            robot.fl.setPower(-0.4);
            robot.bl.setPower(-0.4);
            while(getHeading() > 10){
                if(getHeading() < 25){
                    robot.fl.setPower(-0.15);
                    robot.bl.setPower(-0.15);
                }
            }

            robotAuto.stopDriving();

            robot.fl.setPower(-0.4);
            robot.bl.setPower(-0.4);
            robot.fr.setPower(0.4);
            robot.br.setPower(0.4);
            while(getHeading() > -25){
                if(getHeading() < -10){
                    robot.fl.setPower(-0.15);
                    robot.bl.setPower(-0.15);
                    robot.fr.setPower(0.15);
                    robot.br.setPower(0.15);
                }
            }

            robotAuto.stopAndReset();
            robotAuto.runToPosition();

            robot.fl.setTargetPosition(350);
            robot.bl.setTargetPosition(350);
            robot.fr.setTargetPosition(350);
            robot.br.setTargetPosition(350);

            robot.fl.setPower(0.4);
            robot.bl.setPower(0.4);
            robot.br.setPower(0.4);
            robot.fr.setPower(0.4);
            while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){
            }

            robot.intake.setPower(-1);
            robot.spine.setPower(-1);
            robot.spine.setTargetPosition(-900);
            while(robot.spine.isBusy()){

            }
            sleep(250);
            robot.trapDoor.setPosition(0.13);
            sleep(250);
            robot.spine.setPower(1);
            robot.spine.setTargetPosition(-325);
            robot.inFlip.setPosition(0.15);
            while(robot.spine.isBusy()){
            }

            sleep(700);
            robot.trapDoor.setPosition(1);
            robot.spine.setPower(-1);
            robot.spine.setTargetPosition(-1050);

            robotAuto.runUsing();
            robot.fl.setPower(-0.4);
            robot.bl.setPower(-0.4);
            robot.fr.setPower(0.4);
            robot.br.setPower(0.4);
            while(getHeading() > -40){
                robot.fl.setPower(-0.15);
                robot.bl.setPower(-0.15);
                robot.fr.setPower(0.15);
                robot.br.setPower(0.15);
            }
            robotAuto.stopDriving();

            robotAuto.stopAndReset();
            robotAuto.runToPosition();

            robot.fl.setTargetPosition(-200);
            robot.bl.setTargetPosition(-200);
            robot.fr.setTargetPosition(-200);
            robot.br.setTargetPosition(-200);

            robot.fl.setPower(-0.2);
            robot.bl.setPower(-0.2);
            robot.br.setPower(-0.2);
            robot.fr.setPower(-0.2);
            while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){
            }
            robotAuto.stopDriving();

            //region move to lander
            robotAuto.runUsing();
            while(getHeading() < -2) {
                robot.fr.setPower(-0.3);
                robot.br.setPower(-0.3);
            }
            robotAuto.stopDriving();
            //endregion

            robot.dumpFlip.setPosition(0.1);
            timer1.reset();
            robot.dump.setTargetPosition(-800);
            robot.dump.setPower(-dumpFastPow);
            robot.dumpFlip.setPosition(1);
            while(timer1.seconds() < 0.2){
            }
            robot.rotateArm.setPosition(0.5);
            sleep(2500);*/
            //endregion

            //region right case code

            robotAuto.stopAndReset();
            robotAuto.runUsing();

            robot.fl.setPower(-0.4);
            robot.bl.setPower(-0.4);
            robot.fr.setPower(0.4);
            robot.br.setPower(0.4);
            while(getHeading() > -42){
                if(getHeading() < -20){
                    robot.fl.setPower(-0.15);
                    robot.bl.setPower(-0.15);
                    robot.fr.setPower(0.15);
                    robot.br.setPower(0.15);
                }

            }
            robotAuto.stopDriving();

            robot.inFlip.setPosition(0.65);
            Thread.sleep(50);
            robot.intake.setPower(-1);
            robot.spine.setPower(-1);
            robot.spine.setTargetPosition(-1000);
            while(robot.spine.isBusy()){
                telemetry.addData("spinePos", robot.spine.getCurrentPosition());
                telemetry.addData("spinePow", robot.spine.getPower());
                telemetry.update();

                if(robot.spine.getCurrentPosition() < -950){
                    robot.inFlip.setPosition(0.15);
                }
            }
            robot.spine.setPower(1);
            robot.spine.setTargetPosition(-325);
            robot.trapDoor.setPosition(0.13);
            Thread.sleep(250);
            while(robot.spine.isBusy()){

            }


            robot.fl.setPower(0.35);
            robot.bl.setPower(0.35);
            robot.fr.setPower(-0.35);
            robot.br.setPower(-0.35);

            while(getHeading() < -15){

                if(getHeading() > -35) {
                    robot.fl.setPower(0.15);
                    robot.bl.setPower(0.15);
                    robot.fr.setPower(-0.15);
                    robot.br.setPower(-0.15);
                }
            }
            robotAuto.stopDriving();
            robot.trapDoor.setPosition(1);
            Thread.sleep(100);
            robot.spine.setPower(-1);
            robot.spine.setTargetPosition(-850);

            robotAuto.stopAndReset();
            robotAuto.runToPosition();

            robot.fl.setTargetPosition(-950);
            robot.bl.setTargetPosition(-950);
            robot.fr.setTargetPosition(-950);
            robot.br.setTargetPosition(-950);

            robot.fl.setPower(-0.7);
            robot.bl.setPower(-0.7);
            robot.br.setPower(-0.7);
            robot.fr.setPower(-0.7);

            robot.dumpFlip.setPosition(0.1);
            timer1.reset();
            robot.dump.setTargetPosition(-800);
            robot.dump.setPower(-dumpFastPow);
            while(timer1.seconds() < 0.6){
                if(timer1.seconds() > 0.3){
                    robot.dumpFlip.setPosition(0.9);
                }
            }
            robot.rotateArm.setPosition(0.5);
            robot.dump.setPower(-dumpSlowPow);

            robotAuto.stopDriving();

            Thread.sleep(1000);
            //endregion
            //endregion
        }
        //endregion

        //region first loop

        robot.dump.setPower(0.35);
        robot.dump.setTargetPosition(500);
        sleep(50);

        robot.dumpFlip.setPosition(1);
        robot.rotateArm.setPosition(0.65);

        sleep(200);
        robot.dump.setPower(0.12);

        robot.dumpFlip.setPosition(0.3);

        robotAuto.stopAndReset();
        robotAuto.runToPosition();

        robot.fl.setTargetPosition(1010);
        robot.bl.setTargetPosition(1010);
        robot.fr.setTargetPosition(1010);
        robot.br.setTargetPosition(1010);

        robot.fl.setPower(0.7);
        robot.bl.setPower(0.7);
        robot.br.setPower(0.7);
        robot.fr.setPower(0.7);


        while((robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy())) {
            if(robot.fl.getCurrentPosition() > 550){
                robot.inFlip.setPosition(0.65);
            }

            if(robot.fl.getCurrentPosition() > 550){
                robot.spine.setTargetPosition(-1250);
                robot.spine.setPower(-1);
            }
        }

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fl.setPower(0.35);
        robot.bl.setPower(0.35);
        robot.fr.setPower(-0.35);
        robot.br.setPower(-0.35);
        robot.intake.setPower(-1);

        while(getHeading() < 16){

        }

        robotAuto.stopDriving();
        Thread.sleep(50);

        robot.fl.setPower(-0.25);
        robot.bl.setPower(-0.25);
        robot.fr.setPower(0.25);
        robot.br.setPower(0.25);

        while(getHeading() > 7){


        }

        robotAuto.stopDriving();


        robot.inFlip.setPosition(0.15);
        Thread.sleep(150);

        robot.intake.setPower(1);
        robot.spine.setTargetPosition(-325);
        robot.spine.setPower(1);
        Thread.sleep(50);
        robot.trapDoor.setPosition(0.13);

        robot.intake.setPower(-1);

        robotAuto.stopAndReset();
        robotAuto.runToPosition();

        robot.fl.setTargetPosition(-1070);
        robot.bl.setTargetPosition(-1070);
        robot.fr.setTargetPosition(-1070);
        robot.br.setTargetPosition(-1070);

        robot.fl.setPower(-0.6);
        robot.bl.setPower(-0.6);
        robot.br.setPower(-0.6);
        robot.fr.setPower(-0.6);

        while((robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy())) {

        }

        robot.trapDoor.setPosition(1);
        Thread.sleep(50);
        robot.spine.setTargetPosition(-1000);
        robot.spine.setPower(-1);

        Thread.sleep(150);

        robot.dumpFlip.setPosition(0.1);
        timer1.reset();
        robot.dump.setTargetPosition(-800);
        robot.dump.setPower(-dumpFastPow);
        while(timer1.seconds() < 0.6){
            if(timer1.seconds() > 0.3){
                robot.dumpFlip.setPosition(0.9);
            }
        }
        robot.rotateArm.setPosition(0.5);
        robot.dump.setPower(-dumpSlowPow);

        robotAuto.stopDriving();

        Thread.sleep(1000);
        //endregion

        //region second loop

        robot.dump.setPower(0.35);
        robot.dump.setTargetPosition(500);
        sleep(50);

        robot.dumpFlip.setPosition(1);
        robot.rotateArm.setPosition(0.65);

        sleep(200);
        robot.dump.setPower(0.12);

        robot.dumpFlip.setPosition(0.3);

        robotAuto.stopAndReset();
        robotAuto.runToPosition();

        robot.fl.setTargetPosition(710);
        robot.bl.setTargetPosition(710);
        robot.fr.setTargetPosition(1170);
        robot.br.setTargetPosition(1170);

        robot.fl.setPower(0.7);
        robot.bl.setPower(0.7);
        robot.br.setPower(0.7);
        robot.fr.setPower(0.7);

        while((robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy())) {
            if(robot.fl.getCurrentPosition() > 250){
                robot.inFlip.setPosition(0.65);
                robot.intake.setPower(-1);
            }

            if(robot.fl.getCurrentPosition() > 850){
                robot.spine.setTargetPosition(-1250);
                robot.spine.setPower(-1);
            }
        }


        while(robot.spine.isBusy()){

        }

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fl.setPower(0.35);
        robot.bl.setPower(0.35);
        robot.fr.setPower(-0.35);
        robot.br.setPower(-0.35);
        robot.intake.setPower(-1);

        while(getHeading() < -10){
            if(getHeading() > -30){
                robot.fl.setPower(0.15);
                robot.bl.setPower(0.15);
                robot.fr.setPower(-0.15);
                robot.br.setPower(-0.15);
            }

        }

        robotAuto.stopDriving();

        robot.inFlip.setPosition(0.15);
        Thread.sleep(150);

        robot.intake.setPower(1);
        robot.spine.setTargetPosition(-325);
        robot.spine.setPower(1);
        Thread.sleep(50);
        robot.trapDoor.setPosition(0.13);

        robot.intake.setPower(-1);

        robotAuto.stopAndReset();
        robotAuto.runToPosition();

        robot.fl.setTargetPosition(-1100);
        robot.bl.setTargetPosition(-1100);
        robot.fr.setTargetPosition(-1100);
        robot.br.setTargetPosition(-1100);

        robot.fl.setPower(-0.7);
        robot.bl.setPower(-0.7);
        robot.br.setPower(-0.7);
        robot.fr.setPower(-0.7);

        while((robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy())) {

        }

        robot.trapDoor.setPosition(1);
        Thread.sleep(50);
        robot.spine.setTargetPosition(-1000);
        robot.spine.setPower(-1);
        Thread.sleep(200);


        robot.dumpFlip.setPosition(0.1);
        timer1.reset();
        robot.dump.setTargetPosition(-800);
        robot.dump.setPower(-dumpFastPow);
        while(timer1.seconds() < 0.6){
            if(timer1.seconds() > 0.3){
                robot.dumpFlip.setPosition(0.9);
            }
        }
        robot.rotateArm.setPosition(0.5);
        robot.dump.setPower(-dumpSlowPow);

        robotAuto.stopDriving();

        Thread.sleep(1000);
        //endregion

        //region ending loop
        robot.dump.setPower(0.35);
        robot.dump.setTargetPosition(500);
        sleep(50);

        robot.dumpFlip.setPosition(1);
        robot.rotateArm.setPosition(0.65);

        sleep(250);
        robot.dump.setPower(0.12);

        robot.dumpFlip.setPosition(0.3);


        robotAuto.stopAndReset();
        robotAuto.runToPosition();

        robot.fl.setTargetPosition(800);
        robot.bl.setTargetPosition(800);
        robot.fr.setTargetPosition(800);
        robot.br.setTargetPosition(800);

        robot.fl.setPower(0.7);
        robot.bl.setPower(0.7);
        robot.br.setPower(0.7);
        robot.fr.setPower(0.7);

        robot.spine.setTargetPosition(-1200);
        robot.spine.setPower(-1);

        while((robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy())) {
        }

        //endregion



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

