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
        sleep(400);
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
        robot.fr.setTargetPosition(95);
        robot.bl.setTargetPosition(3000);
        robot.br.setTargetPosition(95);

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

        robot.rotateArm.setPosition(0.6);
        robot.dumpFlip.setPosition(0.35);
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
        robot.hang.setTargetPosition(-2500);
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
                robot.spine.setTargetPosition(-1400);
                robot.dump.setTargetPosition(500);
                robot.dump.setPower(0.2);
                robot.inFlip.setPosition(0.3);
            }
            telemetry.addData("spine",robot.spine.getCurrentPosition());
            telemetry.update();

        }


        robot.fl.setTargetPosition(robot.fl.getCurrentPosition() + 300);//2700 & 1333
        robot.bl.setTargetPosition(robot.bl.getCurrentPosition() + 300);
        robot.fr.setTargetPosition(robot.fr.getCurrentPosition() + 300);
        robot.br.setTargetPosition(robot.br.getCurrentPosition() + 300);

        robot.fl.setPower(0.5);
        robot.bl.setPower(0.5);
        robot.br.setPower(0.5);
        robot.fr.setPower(0.5);

        while(robot.spine.isBusy()){
            if(robot.spine.getCurrentPosition() < -850){
                robot.intake.setPower(1);
            }
        }

        if(robot.spine.getCurrentPosition() < -950){
            robot.intake.setPower(1);
        }

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
        //region rotate to face depot
        robotAuto.runUsing();

        robot.fl.setPower(-0.5);
        robot.bl.setPower(-0.5);
        robot.fr.setPower(0.1);
        robot.br.setPower(0.1);
        while(robotAuto.getHeading() > 95){
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
            while(getHeading() > 35){
                if(getHeading() < 55){
                    robot.fl.setPower(-0.15);
                    robot.bl.setPower(-0.15);
                    robot.fr.setPower(0.15);
                    robot.br.setPower(0.15);
                }
            }

            robotAuto.stopDriving();

            robot.intake.setPower(-1);
            robot.spine.setPower(-1);
            robot.spine.setTargetPosition(-900);
            while(robot.spine.isBusy()){

            }
            robot.spine.setPower(1);
            robot.spine.setTargetPosition(-325);
            robot.inFlip.setPosition(0.15);
            robot.trapDoor.setPosition(0.13);
            while(robot.spine.isBusy()){

            }

            robot.fl.setPower(-0.4);
            robot.bl.setPower(-0.4);
            robot.fr.setPower(0.4);
            robot.br.setPower(0.4);

            while(getHeading() > 4){
                if(getHeading() > 19){
                    robot.fl.setPower(-0.15);
                    robot.bl.setPower(-0.15);
                    robot.fr.setPower(0.15);
                    robot.br.setPower(0.15);
                }
            }
            robotAuto.stopDriving();
            robot.trapDoor.setPosition(1);
            robot.spine.setPower(-1);
            robot.spine.setTargetPosition(-1050);

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
            boolean hi2 = true;

            while((robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()) && (timer1.seconds() < 1.5 || timer1.seconds() > 10)){
                if(robot.fl.getCurrentPosition() < -300){
                    if(hi2) {
                        timer1.reset();
                        hi2 =false;
                    }
                    robot.dump.setTargetPosition(-700);
                    robot.dump.setPower(-0.7);
                }

                if(timer1.seconds() > 0.4){
                    robot.dumpFlip.setPosition(0.95);
                }

                if(robot.dump.getCurrentPosition() < -300){
                    robot.dump.setPower(-0.4);
                    robot.rotateArm.setPosition(0.4);

                }
            }

            robotAuto.stopDriving();

            robot.dumpFlip.setPosition(0.95);
            Thread.sleep(1000);
            //endregion
        } else if(verdict.equals("middle")){
            //region middle case code

            //endregion
        } else if(verdict.equals("right")){
            //region right case code


            //endregion
        }
        //endregion

        //region first loop
        robot.dumpFlip.setPosition(1);
        robot.rotateArm.setPosition(0.6);
        robot.dump.setPower(0.15);
        robot.dump.setTargetPosition(500);
        sleep(400);
        robot.dumpFlip.setPosition(0.35);

        robotAuto.stopAndReset();
        robotAuto.runToPosition();

        robot.fl.setTargetPosition(900);
        robot.bl.setTargetPosition(900);
        robot.fr.setTargetPosition(900);
        robot.br.setTargetPosition(900);

        robot.fl.setPower(0.7);
        robot.bl.setPower(0.7);
        robot.br.setPower(0.7);
        robot.fr.setPower(0.7);

        while((robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy())) {

        }
        robot.inFlip.setPosition(0.65);
        robot.intake.setPower(-1);
        robot.spine.setTargetPosition(-1450);
        robot.spine.setPower(-1);
        while(robot.spine.isBusy()){

        }
        Thread.sleep(150);

        robot.spine.setTargetPosition(-1100);
        robot.spine.setPower(1);
        while(robot.spine.isBusy()){

        }

        robotAuto.stopAndReset();
        robotAuto.runUsing();

        robot.fl.setPower(-0.3);
        robot.bl.setPower(-0.3);
        robot.br.setPower(0.3);
        robot.fr.setPower(0.3);

        while(getHeading() > -15 ){

        }
        robotAuto.stopDriving();

        robot.spine.setTargetPosition(-1400);
        robot.spine.setPower(-1);
        while(robot.spine.isBusy()){

        }
        robot.inFlip.setPosition(0.15);

        robot.fl.setPower(0.15);
        robot.bl.setPower(0.15);
        robot.br.setPower(-0.15);
        robot.fr.setPower(-0.15);

        while(getHeading() < -15){

        }
        robotAuto.stopDriving();

        robot.spine.setTargetPosition(-325);
        robot.spine.setPower(1);

        robotAuto.stopAndReset();
        robotAuto.runToPosition();

        robot.fl.setTargetPosition(-900);
        robot.bl.setTargetPosition(-900);
        robot.fr.setTargetPosition(-900);
        robot.br.setTargetPosition(-900);

        robot.fl.setPower(-0.7);
        robot.bl.setPower(-0.7);
        robot.br.setPower(-0.7);
        robot.fr.setPower(-0.7);

        while((robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy())) {
            if(robot.fl.getCurrentPosition() < -300){
                robot.trapDoor.setPosition(0.13);
            }
        }
        Thread.sleep(50);
        robot.trapDoor.setPosition(1);
        robot.spine.setTargetPosition(-1000);
        robot.spine.setPower(-1);
        Thread.sleep(200);

        robot.dump.setTargetPosition(-700);
        robot.dump.setPower(-0.7);
        robot.dumpFlip.setPosition(0.1);
        Thread.sleep(200);
        robot.dumpFlip.setPosition(0.95);
        Thread.sleep(150);
        robot.rotateArm.setPosition(0.4);
        while(robot.dump.isBusy()){
            if(robot.dump.getCurrentPosition() < -300){
                robot.dump.setPower(-0.4);
            }
        }
        Thread.sleep(750);
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

