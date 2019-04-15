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

@Autonomous(name="worldsDepot")
public class worldsDepot extends LinearOpMode {
    roverHMAP robot;
    roverAuto robotAuto;
    private VisionThing vision;


    public void runOpMode() throws InterruptedException {
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
        robot.trapDoor.setPosition(1);
        robot.inFlip.setPosition(0.15);
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

        //region driveForward deposit and drive back
        robot.spine.setTargetPosition(-1200);
        robot.spine.setPower(-1);

        robotAuto.stopAndReset();
        robotAuto.runToPosition();

        robot.fl.setTargetPosition(1050);
        robot.bl.setTargetPosition(1050);
        robot.fr.setTargetPosition(1050);
        robot.br.setTargetPosition(1050);

        robot.fl.setPower(0.45);
        robot.bl.setPower(0.45);
        robot.br.setPower(0.45);
        robot.fr.setPower(0.45);

        while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){


        }
        robot.intake.setPower(1);
        Thread.sleep(1500);
        robot.intake.setPower(0);

        robot.spine.setTargetPosition(-325);
        robot.spine.setPower(1);
        robot.dump.setTargetPosition(500);
        robot.dump.setPower(0.2);
        robot.inFlip.setPosition(0.3);

        robotAuto.stopAndReset();
        robotAuto.runToPosition();

        robot.fl.setTargetPosition(-750);
        robot.bl.setTargetPosition(-750);
        robot.fr.setTargetPosition(-750);
        robot.br.setTargetPosition(-750);

        robot.fl.setPower(-0.45);
        robot.bl.setPower(-0.45);
        robot.br.setPower(-0.45);
        robot.fr.setPower(-0.45);

        while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){


        }
        Thread.sleep(50);
        robot.inFlip.setPosition(0.67);
        //endregion

        //region case code
        if(verdict.equals("left")){
            //region left case code
            robotAuto.stopAndReset();
            robotAuto.runUsing();

            while(getHeading() < 15){
                robot.fl.setPower(0.2);
                robot.bl.setPower(0.2);
                robot.fr.setPower(-0.2);
                robot.br.setPower(-0.2);
            }

            robotAuto.stopDriving();

            robot.intake.setPower(-1);
            robot.spine.setTargetPosition(-1000);
            robot.spine.setPower(-1);

            while(robot.spine.isBusy()){

            }

            robot.inFlip.setPosition(0.15);
            robot.trapDoor.setPosition(0.13);
            Thread.sleep(250);
            robot.spine.setTargetPosition(-325);
            robot.spine.setPower(1);

            while(robot.spine.isBusy()){

            }

            while(getHeading() > 3){
                robot.fl.setPower(-0.1);
                robot.bl.setPower(-0.1);
                robot.fr.setPower(0.1);
                robot.br.setPower(0.1);
            }

            robotAuto.stopAndReset();
            robotAuto.runToPosition();

            robot.fl.setTargetPosition(-450);
            robot.bl.setTargetPosition(-450);
            robot.fr.setTargetPosition(-450);
            robot.br.setTargetPosition(-450);

            robot.fl.setPower(-0.45);
            robot.bl.setPower(-0.45);
            robot.br.setPower(-0.45);
            robot.fr.setPower(-0.45);

            while(robot.fl.isBusy() || robot.fr.isBusy() || robot.bl.isBusy() || robot.br.isBusy()){

            }

            robot.spine.setPower(-1);
            robot.spine.setTargetPosition(-850);
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

            //endregion
        } else if(verdict.equals("middle")){

        } else if(verdict.equals("right")){
            //region right case code
            robotAuto.stopAndReset();
            robotAuto.runUsing();

            while(getHeading() > -15){
                robot.fl.setPower(-0.25);
                robot.bl.setPower(-0.25);
                robot.fr.setPower(0.25);
                robot.br.setPower(0.25);
            }

            robotAuto.stopDriving();

            robot.intake.setPower(-1);
            robot.spine.setTargetPosition(-1000);
            robot.spine.setPower(-1);
            //endregion
        }
        //endregion

        //region drive back and dump code

        //endregion

        //region path to crater

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

