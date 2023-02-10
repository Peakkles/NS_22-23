//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//import java.util.ArrayList;
//
//@Autonomous(name="TimeAuto")
//public class TimeAuto extends LinearOpMode {
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftFrontDrive = null;
//
//    private DcMotor leftBackDrive = null;
//    private DcMotor rightFrontDrive = null;
//    private DcMotor rightBackDrive = null;
//    //    private DcMotor slide = null;
//    double wristPos;
//    double armLeftPos;
//    double armRightPos;
//    double clawPos;
//    private Servo wrist;
//    private Servo armLeft;
//    private Servo armRight;
//    private Servo claw;
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    static final double FEET_PER_METER = 3.28084;
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//    // Tag ID 1,2,3 from the 36h11 family
//    int LEFT = 1;
//    int MIDDLE = 3;
//    int RIGHT = 2;
//
//    AprilTagDetection tagOfInterest = null;
//
//    double motorCorrection = 1.28;
//
//    @Override
//    public void runOpMode() {
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeft");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRight");
//        armRight = hardwareMap.get(Servo.class, "ArmRight");
//        armLeft = hardwareMap.get(Servo.class, "ArmLeft");
//        claw = hardwareMap.get(Servo.class, "Claw");
//        wrist = hardwareMap.get(Servo.class, "Wrist");
//
//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        armRightPos = 1;
//        armLeftPos = 0.0;
//        clawPos = 0.8;
//        wristPos = 0;
//
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//
//        /*
//         * The INIT-loop:
//         * This REPLACES waitForStart!
//         */
//        while (!isStarted() && !isStopRequested()) {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if (currentDetections.size() != 0) {
//                boolean tagFound = false;
//
//                for (AprilTagDetection tag : currentDetections) {
//                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if (tagFound) {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                } else {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if (tagOfInterest == null) {
//                        telemetry.addLine("(The tag has never been seen)");
//                    } else {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            } else {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if (tagOfInterest == null) {
//                    telemetry.addLine("(The tag has never been seen)");
//                } else {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.update();
//            sleep(20);
//        }
//
//        /*
//         * The START command just came in: now work off the latest snapshot acquired
//         * during the init loop.
//         */
//
//        /* Update the telemetry */
//        if (tagOfInterest != null) {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        } else {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }
//
////        This is the actual movement part:
//        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
//            //trajectory:
//            forwardByTime(0.4, 2500);
//            sleep(500);
//            rightByTime(-0.4, 1000);
//        } else if (tagOfInterest.id == MIDDLE) {
//            //trajectory:
//            forwardByTime(0.4, 2500);
//        } else {
//            //trajectory:
//            forwardByTime(0.4, 1500);
//            rightByTime(0.4, 700);
//        }
//
//    }
//
//    void tagToTelemetry(AprilTagDetection detection) {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
//
//
//// positive power goes forwards
//    void forwardByTime(double power, long time){
//        leftFrontDrive.setPower(-power * motorCorrection + 0); // potentially *2 or +0.1
//        leftBackDrive.setPower(-power * motorCorrection + 0); // potentially *2 or +0.1
//        rightFrontDrive.setPower(-power);
//        rightBackDrive.setPower(-power);
//
//        sleep(time);
//        driveStop();
//    }
////    positive power goes right
//    void rightByTime(double power, long time) {
//        leftFrontDrive.setPower(power * motorCorrection + 0); // potentially *2 or +0.1
//        leftBackDrive.setPower(-power * motorCorrection + 0); // potentially *2 or +0.1
//        rightFrontDrive.setPower(-power);
//        rightBackDrive.setPower(power);
//
//        sleep(time);
//        driveStop();
//    }
//
//    void driveStop() {
//        leftFrontDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightBackDrive.setPower(0);
//    }
//
//
//
//}