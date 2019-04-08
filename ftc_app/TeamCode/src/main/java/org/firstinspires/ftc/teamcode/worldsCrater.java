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


    public void runOpMode() throws InterruptedException {
        robot = new roverHMAP();
        robot.init(hardwareMap,true);
        robotAuto = new roverAuto(robot);

        robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.dump.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,new PIDCoefficients(6,0,0));
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
        sleep(200);
        robot.dumpFlip.setPosition(0);
        sleep(100);
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
        while(robot.hang.getCurrentPosition() > -4370) {
        }
        sleep(10);
        robot.hang.setPower(0);
        //endregion


        vision.disable();

        //region pid coefficients
        double p = 12;
        double i = 2;
        double d = 2;

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
        robot.fr.setPower(0.03);
        robot.br.setPower(0.03);

        robot.fl.setTargetPositionTolerance(10);
        robot.bl.setTargetPositionTolerance(10);
        robot.fr.setTargetPositionTolerance(10);
        robot.br.setTargetPositionTolerance(10);


        //12 0.2 1
        //12 2 2
        //12 0.5 1.5

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
        Thread.sleep(10);
        robotAuto.runToPosition();
        Thread.sleep(10);

        telemetry.addData("fl",robot.fl.getCurrentPosition());
        telemetry.addData("bl",robot.bl.getCurrentPosition());
        telemetry.addData("br",robot.br.getCurrentPosition());
        telemetry.addData("fr",robot.fr.getCurrentPosition());
        telemetry.addData("fl",robot.fl.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
        telemetry.update();

        robot.fl.setTargetPosition(2700);
        robot.bl.setTargetPosition(2700);
        robot.fr.setTargetPosition(1333);
        robot.br.setTargetPosition(1333);

        robot.fl.setPower(0.9);
        robot.bl.setPower(0.9);
        robot.br.setPower(0.5);
        robot.fr.setPower(0.5);

        while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){
            if(robot.fl.getCurrentPosition() > 1800){
                robot.spine.setPower(-1);
                robot.spine.setTargetPosition(-1100);
            }

        }

        robotAuto.verticalDrive(0);
        while(robot.spine.isBusy()){

        }

        robot.spine.setPower(1);
        robot.spine.setTargetPosition(-325);
        //endregion

        sleep(400);
        //region rotate to face depot
        robotAuto.runUsing();

        robot.fl.setPower(-0.5);
        robot.bl.setPower(-0.5);
        robot.fr.setPower(0.1);
        robot.br.setPower(0.1);
        while(robotAuto.getHeading() > 98){

        }

        robotAuto.stopDriving();
        Thread.sleep(50);
        //endregion

        //region drive back
        robotAuto.stopAndReset();
        Thread.sleep(10);
        robotAuto.runToPosition();
        Thread.sleep(10);

        robot.fl.setTargetPosition(-2200);
        robot.bl.setTargetPosition(-2200);
        robot.fr.setTargetPosition(-2200);
        robot.br.setTargetPosition(-2200);

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

