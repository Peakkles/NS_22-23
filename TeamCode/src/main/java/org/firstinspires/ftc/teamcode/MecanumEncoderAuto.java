package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.lang.Math;

@Autonomous(name="MecanumEncoderAuto")
public class MecanumEncoderAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack = null;

    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    //    private DcMotor slide = null;
    double wristPos;
    double armLeftPos;
    double armRightPos;
    double clawPos;
    private Servo rightArm;
    private Servo wrist;
    private Servo leftArm;
    private Servo claw;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 3;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;

    double motorCorrection = 1.25;

    @Override
    public void runOpMode() {
        leftBack = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightBack = hardwareMap.get(DcMotor.class, "BackLeft");
        leftFront = hardwareMap.get(DcMotor.class, "FrontRight");
        rightFront = hardwareMap.get(DcMotor.class, "BackRight");
        leftArm = hardwareMap.get(Servo.class, "ArmRight");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        claw = hardwareMap.get(Servo.class, "Claw");
        rightArm = hardwareMap.get(Servo.class, "ArmLeft");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        armRightPos = 1;
        armLeftPos = 0.0;
        clawPos = 0;
        wristPos = 0;


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

//        This is the actual movement part:
//        cycle cones here:


//        park based on april tag here:
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            //trajectory:
            forwardByEncoder(900, 0.3);
            sleep(500);
            rightByEncoder(800, -0.4);
            sleep(500);
//            forwardByEncoder(200, 0.3);
            //left
        } else if (tagOfInterest.id == MIDDLE) {
            //trajectory:
            forwardByEncoder(980, 0.3);
        } else {
            //trajectory:
            forwardByEncoder(800, 0.3);
            sleep(500);
            rightByEncoder(880, 0.3);
            //right
        }



    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

//    positive power goes forward, negative power goes backward
    void forwardByEncoder(int inches, double power) {
//        inches *= some number (find this number by testing on field)

        inches *= 1;
        if(opModeIsActive()) {
            leftBack.setPower(power * 1);
            rightBack.setPower(power * 1);
            leftFront.setPower(-power * 1);
            rightFront.setPower(power * 1);

            while(opModeIsActive() && Math.abs(leftBack.getCurrentPosition()) < inches){
//                do nothing
                telemetry.addData("leftFront currentPos: ", leftBack.getCurrentPosition());
                telemetry.update();
            }

            driveStop(); // this might be unnecessary

            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

//    positive power moves right, negative power moves left
    void rightByEncoder(int inches, double power) {
//        inches *= some number (find this number by testing on field)
        inches *= 1;
        if(opModeIsActive()) {
            leftBack.setPower(power * 1); // potentially *2 or +0.1
            rightBack.setPower(-power * 1); // potentially *2 or +0.1
            leftFront.setPower(-power * 1);
            rightFront.setPower(power * 1);

            while(opModeIsActive() && Math.abs(leftBack.getCurrentPosition()) < inches) {
//                do nothing
                telemetry.addData("leftFront currentPos: ", leftBack.getCurrentPosition());
                telemetry.update();
            }
            driveStop(); // this might be unnecessary

            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

    void turnClockwise(int inches, double power) {
//        inches *= some number (find this number by testing on field)
        inches *= 1;
        if(opModeIsActive()) {
            leftBack.setPower(power * 1);
            rightBack.setPower(-power * 1);
            leftFront.setPower(-power * 1);
            rightFront.setPower(-power * 1);

            while(opModeIsActive() && Math.abs(leftBack.getCurrentPosition()) < inches) {
//                do nothing
                telemetry.addData("leftFront currentPos: ", leftBack.getCurrentPosition());
                telemetry.update();
            }
            driveStop(); // this might be unnecessary

            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }


    //    Dont use this
    void diagonalByEncoder(int xInches, int yInches, double power) {
        xInches*=-1;
        yInches*=-1;

        double max = Math.max(Math.abs(yInches-xInches), Math.abs(yInches+xInches));
        leftBack.setPower(power * (yInches - xInches)/max);
        leftFront.setPower(power * (yInches + xInches)/max);
        rightBack.setPower(power * (yInches + xInches)/max);
        rightFront.setPower(power * (yInches - xInches)/max);

    }

    void forwardByTime(double power, long time){
        leftBack.setPower(power);
        rightBack.setPower(power);
        leftFront.setPower(-power);
        rightFront.setPower(power);

        sleep(time);
        driveStop();
    }

    void rightByTime(double power, long time) {
        leftBack.setPower(power);
        rightBack.setPower(-power);
        leftFront.setPower(-power);
        rightFront.setPower(power);

        sleep(time);
        driveStop();
    }

    void driveStop() {
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }



}