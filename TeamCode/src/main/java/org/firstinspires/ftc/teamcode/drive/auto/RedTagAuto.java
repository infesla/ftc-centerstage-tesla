package org.firstinspires.ftc.teamcode.drive.auto;

import android.util.Size;

import java.util.concurrent.TimeUnit;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.methods.PixelsControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(group = "drive")
public class RedTagAuto extends LinearOpMode {
    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotorEx rightFront;

    public PixelsControl PixelsControl = new PixelsControl();
    public ElapsedTime runtime = new ElapsedTime();
    private boolean isDetected = false;

    private int drive_id = 4;

    private WebcamName webcam1, webcam2;

    private enum Position {
        LEFT,
        CENTER,
        RIGHT
    }

    public Position position;

    double x = 0;
    private static final String TFOD_MODEL_ASSET = "red_TSE_CENTERSTAGE_v01.tflite";
    private static final String[] LABELS = {
            "red TSE",
    };

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    double STRAFE_GAIN = 0.02;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    double TURN_GAIN = 0.01;    //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    double MAX_AUTO_SPEED = 0.7;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_STRAFE = 0.7;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_TURN = 0.4;   //  Clip the turn speed to this max value (adjust for your robot)


    @Override
    public void runOpMode() throws InterruptedException {

        //RedTagAuto processing

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        tagProcessor.setDecimation(2);

        //TensorFlow processing

        TfodProcessor tfodProcessor = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        //VisionPortal processing

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tfodProcessor)
                .addProcessor(tagProcessor)
                .setCamera(switchableCamera)
                .setCameraResolution(new Size(640, 480))
                .build();


        while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            sleep(20);
        }

        AprilTagDetection tag = null;

        visionPortal.setActiveCamera(webcam2);

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure((long) 14, TimeUnit.MILLISECONDS);

        //Hardware init

        PixelsControl.initHW(this);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        //Start

        runtime.reset();

        List<Recognition> currentRecognitions = tfodProcessor.getRecognitions();

        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;

            telemetry.addData("X", x);
            telemetry.update();

        }

        visionPortal.setProcessorEnabled(tfodProcessor, false);

        //location
        if (x < 150) {
            position = Position.LEFT;
        }
        else if (x >= 150 && x < 450) {
            position = Position.CENTER;
        }
        else {
            position = Position.RIGHT;
        }

        visionPortal.setActiveCamera(webcam1);

        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure((long) 3, TimeUnit.MILLISECONDS);

        runtime.reset();

        while (!isStopRequested() && ((runtime.milliseconds() - 700) < 0)) {
            leftFront.setPower(-0.6);
            leftRear.setPower(0.7);
            rightFront.setPower(0.7);
            rightRear.setPower(-0.6);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        sleep(1000);

        //changed

        runtime.reset();

        while (runtime.milliseconds() < 2000 && !isStopRequested()) {

            List<AprilTagDetection> myAprilTagDetections = tagProcessor.getDetections();

            isDetected = false;
            for (AprilTagDetection detection : myAprilTagDetections) {
                if (detection.id == drive_id) {  // This check for non-null Metadata is not needed for reading only ID code.
                    tag = detection;
                    isDetected = true;
                    break;
                }
            }

            double rangeError      = (tag.ftcPose.range - 30);
            double headingError    = (tag.ftcPose.bearing);
            double yawError        = tag.ftcPose.yaw;

            calcRobot(rangeError, headingError, yawError, tag);

        }

        sleep(500);

        SPEED_GAIN = 0.035;
        STRAFE_GAIN = 0.035;
        TURN_GAIN = 0.01;

        if (position == Position.CENTER) {

            SPEED_GAIN = 0.035;
            STRAFE_GAIN = 0.15;
            TURN_GAIN = 0.01;

            runtime.reset();
            while (runtime.milliseconds() < 600 && !isStopRequested()) {
                List<AprilTagDetection> myAprilTagDetections = tagProcessor.getDetections();

                isDetected = false;
                for (AprilTagDetection detection : myAprilTagDetections) {
                    if (detection.id == drive_id) {  // This check for non-null Metadata is not needed for reading only ID code.
                        tag = detection;
                        isDetected = true;
                        break;
                    }
                }

                double rangeError = (tag.ftcPose.range - 35);
                double yawError = tag.ftcPose.yaw;
                double xError = tag.ftcPose.x - 15;

                calcRobot(rangeError, 0, xError, tag);
            }
        }
        else if (position == Position.LEFT) {

            runtime.reset();

            while (runtime.milliseconds() < 2000 && !isStopRequested()) {

                List<AprilTagDetection> myAprilTagDetections = tagProcessor.getDetections();

                isDetected = false;
                for (AprilTagDetection detection : myAprilTagDetections) {
                    if (detection.id == drive_id) {  // This check for non-null Metadata is not needed for reading only ID code.
                        tag = detection;
                        isDetected = true;
                        break;
                    }
                }

                double rangeError = (tag.ftcPose.range - 53);
                double headingError = (tag.ftcPose.bearing);
                double yawError = tag.ftcPose.yaw;

                calcRobot(rangeError, headingError, yawError, tag);
            }
        }
        else {

            runtime.reset();

            while (runtime.milliseconds() < 2000 && !isStopRequested()) {

                List<AprilTagDetection> myAprilTagDetections = tagProcessor.getDetections();

                isDetected = false;
                for (AprilTagDetection detection : myAprilTagDetections) {
                    if (detection.id == drive_id) {  // This check for non-null Metadata is not needed for reading only ID code.
                        tag = detection;
                        isDetected = true;
                        break;
                    }
                }

                double rangeError      = (tag.ftcPose.range - 25);
                double headingError    = (tag.ftcPose.bearing);
                double yawError        = tag.ftcPose.yaw;

                calcRobot(rangeError, headingError, yawError, tag);

            }
        }

        runtime.reset();

        SPEED_GAIN = 0.035;
        STRAFE_GAIN = 0.035;
        TURN_GAIN = 0.01;

        sleep(250);

        PixelsControl.setSuction(0.6);

        sleep(1000);

        PixelsControl.setSuction(0);

        runtime.reset();

        while (runtime.milliseconds() < 1000 && !isStopRequested()) {

            List<AprilTagDetection> myAprilTagDetections = tagProcessor.getDetections();

            isDetected = false;
            for (AprilTagDetection detection : myAprilTagDetections) {
                if (detection.id == drive_id) {  // This check for non-null Metadata is not needed for reading only ID code.
                    tag = detection;
                    isDetected = true;
                    break;
                }
            }

            double rangeError      = (tag.ftcPose.range - 30);
            double headingError    = (tag.ftcPose.bearing);
            double yawError        = tag.ftcPose.yaw;

            calcRobot(rangeError, headingError, yawError, tag);

        }

        sleep(250);

        SPEED_GAIN = 0.035;
        STRAFE_GAIN = 0.035;
        TURN_GAIN = 0.01; //changed

        int end_id = 0;
        if (position == Position.CENTER) {
            end_id = 5;
        }
        else if (position == Position.LEFT) {
            end_id = 4;
        }
        else {
            end_id = 6;
        }

        runtime.reset();

        while (runtime.milliseconds() < 1000 && !isStopRequested()) {

            List<AprilTagDetection> myAprilTagDetections = tagProcessor.getDetections();

            isDetected = false;
            for (AprilTagDetection detection : myAprilTagDetections) {
                if (detection.id == end_id) {  // This check for non-null Metadata is not needed for reading only ID code.
                    tag = detection;
                    isDetected = true;
                    break;
                }
            }

            double rangeError      = (tag.ftcPose.range - 15);
            double headingError    = (tag.ftcPose.bearing);
            double yawError        = tag.ftcPose.yaw;

            calcRobot(rangeError, headingError, yawError, tag);

        }

        runtime.reset();

        while (runtime.milliseconds() < 1000 && !isStopRequested()) {

            List<AprilTagDetection> myAprilTagDetections = tagProcessor.getDetections();

            isDetected = false;
            for (AprilTagDetection detection : myAprilTagDetections) {
                if (detection.id == end_id) {  // This check for non-null Metadata is not needed for reading only ID code.
                    tag = detection;
                    isDetected = true;
                    break;
                }
            }

            double rangeError      = (tag.ftcPose.range - 2);
            double headingError    = (tag.ftcPose.bearing);
            double yawError        = tag.ftcPose.yaw;

            calcRobot(rangeError, headingError, yawError, tag);

        }

        PixelsControl.setTele(1);

        sleep(900);

        PixelsControl.setTele(0);

        PixelsControl.setFlip(0.65);

        runtime.reset();

        while (!isStopRequested() && ((runtime.milliseconds() - 800) < 0)) {
            leftFront.setPower(0.3);
            leftRear.setPower(0.25);
            rightFront.setPower(0.3);
            rightRear.setPower(0.25);
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        runtime.reset();

        sleep(500);

        PixelsControl.setHook(0.3);

        sleep(1000);

        PixelsControl.setTele(1);

        sleep(500);

        PixelsControl.setTele(0);

        PixelsControl.setHook(0.795);

        sleep(1000);

        runtime.reset();

        while (!isStopRequested() && ((runtime.milliseconds() - 300) < 0)) {
            leftFront.setPower(-0.3);
            leftRear.setPower(-0.3);
            rightFront.setPower(-0.3);
            rightRear.setPower(-0.3);
        }

        sleep(500);


        runtime.reset();

        while (!isStopRequested() && ((runtime.milliseconds() - 500) < 0)) {
            leftFront.setPower(-0.4);
            leftRear.setPower(0.4);
            rightFront.setPower(0.4);
            rightRear.setPower(-0.4);
        }

        PixelsControl.setFlip(0.03);

        PixelsControl.setTele(-1);

        sleep(500);

        PixelsControl.setTele(0);

        visionPortal.close();

    }

    public void calcRobot(double rangeError, double headingError, double yawError, AprilTagDetection tag) {

        double drive = 0;
        double strafe = 0;
        double turn = 0;

        // Use the speed and turn "gains" to calculate how we want the robot to move.

        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        if (isDetected) {
            moveRobot(drive, strafe, turn);
        }
        else {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        }

        telemetry.addData("TSE loc X", x);
        telemetry.addData("TSE pos", position);

        telemetry.addData("X", tag.ftcPose.x);
        telemetry.addData("Y", tag.ftcPose.y);
        telemetry.addData("Z", tag.ftcPose.z);

        telemetry.addData("Roll", tag.ftcPose.roll);
        telemetry.addData("Pinch", tag.ftcPose.pitch);
        telemetry.addData("Yaw", tag.ftcPose.yaw);
        telemetry.addData("Bearing", tag.ftcPose.bearing);

        telemetry.addData("ERROR r", rangeError);
        telemetry.addData("ERROR h", headingError);
        telemetry.addData("ERROR y", yawError);

        sleep(10);

        telemetry.update();
    }

    public void moveRobot(double x, double y, double yaw) {

        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftRearPower     =  x +y -yaw;
        double rightRearPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftRearPower /= max;
            rightRearPower /= max;
        }

        // Send powers to the wheels.
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }
}
