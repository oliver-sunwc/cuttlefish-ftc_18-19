package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import java.util.List;
import java.util.Locale;

@Autonomous(name="autDepotProfileBetterIdea")
public class autoDepotSideProfileCopyMiddle extends LinearOpMode {
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
        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int currPos = robot.hang.getCurrentPosition();

        robot.hang.setPower(1);
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



        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hang.setPower(0);
        Thread.sleep(10);
        //endregion

        //region set dump arm up
        robot.flipLArm.setPosition(0.35);
        robot.flipRArm.setPosition(0.65);

        robot.flap.setPosition(0.5);
        //endregion
        vision.disable();


        // region move forward 35 cm
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
        while(robot.dist.getDistance(DistanceUnit.CM)< 25){
            telemetry.addData("dist",robot.dist.getDistance(DistanceUnit.CM));
            telemetry.update();
            if(robot.dist.getDistance(DistanceUnit.CM) > 20){
                robot.spine.setPower(-1);
            }
        }
        //endregion

        //region set dump arm and hang down and rotateArm flat
        robot.flipLArm.setPosition(1);
        robot.flipRArm.setPosition(0);
        robot.rotateArm.setPosition(0.75);

        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hang.setPower(1);
        //endregion

        //region set intake flip down, stop hang and spine, start intake
        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.inFlip.setPower(0.65);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 350);
        Thread.sleep(300);



        robot.intake.setPower(-0.7);
        Thread.sleep(150);
        robot.hang.setPower(0);
        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Thread.sleep(150);

        robot.inFlip.setPower(0);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.spine.setPower(0);
        Thread.sleep(200);
        //endregion

        //region bring spine back and intake up
        robot.spine.setPower(1);
        Thread.sleep(250);
        robot.intake.setPower(0);

        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.inFlip.setPower(-0.85);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition()-350);
        Thread.sleep(500);

        //endregion

        // region move back to 10 cm away

        robotAuto.verticalDrive(-0.1);
        while(robot.dist.getDistance(DistanceUnit.CM)> 15){
            telemetry.addData("dist",robot.dist.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() - 1);
        robot.inFlip.setPower(-0.1);
        //endregion


        if(verdict.equals("left")){
            while(getHeading() < 35){
                    robot.fl.setPower(0.1);
                    robot.bl.setPower(0.1);
                    robot.fr.setPower(-0.1);
                    robot.br.setPower(-0.1);
            }

            robotAuto.stopDriving();
        } else if(verdict.equals("right")){
            while(getHeading() > -35){
                robot.fl.setPower(-0.1);
                robot.bl.setPower(-0.1);
                robot.fr.setPower(0.1);
                robot.br.setPower(0.1);
            }

            robotAuto.stopDriving();
        } else{

        }

        //region drop intake and extend spine while moving back a little

        robot.spine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.inFlip.setPower(0.65);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() + 350);
        Thread.sleep(250);

        robot.inFlip.setPower(0);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.intake.setPower(0.3);
        robot.spine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.spine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.spine.setPower(-1);
        int initPos = robot.spine.getCurrentPosition();


        ElapsedTime timer1 = new ElapsedTime();
        timer1.startTime();
        while(robot.spine.getCurrentPosition()-initPos > -500 && timer1.seconds() < 2.0){
            telemetry.addData("thing",robot.intake.getCurrentPosition());
            telemetry.addData("encoder",robot.spine.getCurrentPosition());
            telemetry.update();


        }

        robot.spine.setPower(0);
        hi2.reset();


        robotAuto.stopDriving();

        robot.fl.setPower(-0.25);
        robot.bl.setPower(-0.25);
        robot.fr.setPower(0.25);
        robot.br.setPower(0.25);
        Thread.sleep(150);

        robot.fl.setPower(0.25);
        robot.bl.setPower(0.25);
        robot.fr.setPower(-0.25);
        robot.br.setPower(-0.25);
        Thread.sleep(300);

        robot.fl.setPower(-0.25);
        robot.bl.setPower(-0.25);
        robot.fr.setPower(0.25);
        robot.br.setPower(0.25);
        Thread.sleep(150);

        robotAuto.stopDriving();

        //endregion

        //region bring intake up to transfer and spine back

        Thread.sleep(250);
        robot.inFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.inFlip.setPower(-0.85);
        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition() - 350);
        robot.spine.setPower(1);
        Thread.sleep(300);
        robot.intake.setPower(0);
        Thread.sleep(300);
        robot.spine.setPower(0);
        robot.intake.setPower(0.4);

        robot.inFlip.setTargetPosition(robot.inFlip.getCurrentPosition());
        robot.inFlip.setPower(-0.05);
        //endregion

        if(verdict.equals("right")){
            while(getHeading() < -0.5){
                robot.fl.setPower(0.1);
                robot.bl.setPower(0.1);
                robot.fr.setPower(-0.1);
                robot.br.setPower(-0.1);
            }

            robotAuto.stopDriving();
        } else if(verdict.equals("left")){
            while(getHeading() > 0.5){
                robot.fl.setPower(-0.1);
                robot.bl.setPower(-0.1);
                robot.fr.setPower(0.1);
                robot.br.setPower(0.1);
            }

            robotAuto.stopDriving();
        } else{

        }

        //region move back
        robotAuto.verticalDrive(-0.4);
        while(robot.dist.getDistance(DistanceUnit.CM) > 6){
            if(robot.dist.getDistance(DistanceUnit.CM) > 20){
            }
        }

        robotAuto.verticalDrive(0.02);
        Thread.sleep(50);
        robotAuto.stopDriving();
        robot.intake.setPower(0);
        //endregion

        //region extend spine and flip up to dump
        robot.hang.setPower(-0.05);
        robot.spine.setPower(-1);
        robot.flipLArm.setPosition(0);
        robot.flipRArm.setPosition(1);
        robot.rotateArm.setPosition(0.35);
        Thread.sleep(300);
        robot.hang.setPower(0);
        Thread.sleep(200);
        robot.spine.setPower(0);
        Thread.sleep(500);
        //endregion


        /*
        //region extend spine and flip up to dump
        robot.hang.setPower(-0.05);
        robot.spine.setPower(-1);
        robot.flipLArm.setPosition(0);
        robot.flipRArm.setPosition(1);
        robot.rotateArm.setPosition(0.35);
        Thread.sleep(300);
        robot.hang.setPower(0);
        Thread.sleep(200);
        robot.spine.setPower(0);
        Thread.sleep(500);
        //endregion*/




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

