package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.mecanism.Lift;

@TeleOp(name="Mecanum TeleOp", group="Linear Opmode")

public class Mecanum123 extends LinearOpMode {
    Lift lift = new Lift();


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack = null;

    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
        private DcMotor slide = null;
    double rightArmPos;
    private Servo rightArm;
    private Servo wrist;
    private Servo leftArm;
    boolean bWasDown = false;
    double wristPosMaybe;
    double leftArmPos;
    double clawPosMaybe;
    boolean YPressed = false;
    boolean XPressed = false;
    boolean armUp = false;
    boolean slow = false;
    boolean clawShut = false;
    boolean BPressed = false;
    boolean armHigh = true;
    boolean aPressed = false;

    private Servo claw;
    boolean xWasDown = false;

    double motorCorrection = 1;

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftBack = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightBack = hardwareMap.get(DcMotor.class, "BackLeft");
        leftFront = hardwareMap.get(DcMotor.class, "FrontRight");
        rightFront = hardwareMap.get(DcMotor.class, "BackRight");

//        left and right arms are named when looking at the robot from the front
        leftArm = hardwareMap.get(Servo.class, "ArmLeft");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        claw = hardwareMap.get(Servo.class, "Claw");
        rightArm = hardwareMap.get(Servo.class, "ArmRight");
//        armLeft = hardwareMap.get(Servo.class, "LeftArm");
//        armRight = hardwareMap.get(Servo.class, "RightArm");

//        lift.init(hardwareMap);
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        slide.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftArmPos = 1; // actually right arm, start at 1
        wristPosMaybe = 0.58;
        clawPosMaybe = 0.4;
        rightArmPos = 0; // actually left arm, start at 0


        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = gamepad1.left_stick_y*0.5;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x*0.5;
        double yaw = gamepad1.right_stick_x*0.5;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            if (gamepad1.b) {
                telemetry.addData("Status of B", "B is Pressed ");
                telemetry.update();
                if(!BPressed) {
                    if(!slow){
                        telemetry.addData("Status", "If Statement Entered");
                        telemetry.update();

                        slow=true;
                    } else {
                        telemetry.addData("Status", "Else Statement Entered");
                        telemetry.update();

                        ;
                        slow=false;
                    }
                    BPressed=true;
                }
            }else{
                BPressed = false;
            }
            if (slow) {
                axial = gamepad1.left_stick_y*0.2;  // Note: pushing stick forward gives negative value
                lateral = gamepad1.left_stick_x*0.2;
                yaw = gamepad1.right_stick_x*0.2;
            }

            if(!slow) {
                axial = gamepad1.left_stick_y*0.5;  // Note: pushing stick forward gives negative value
                lateral = gamepad1.left_stick_x*0.5;
                yaw = gamepad1.right_stick_x*0.5;
            }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double backLeftPower = (axial + lateral - yaw);
            double frontLeftPower = (axial - lateral - yaw);
            double backRightPower = (axial - lateral + yaw);
            double frontRightPower = (axial + lateral + yaw);

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(backLeftPower), Math.abs(frontLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            max = Math.max(max, Math.abs(frontRightPower));

            if (max > 1.0) {
                backLeftPower /= max;
                frontLeftPower /= max;
                backRightPower /= max;
                frontRightPower /= max;
            }

            // Send calculated power to wheels
            leftBack.setPower(-backLeftPower); // actually Back Left
            leftFront.setPower(frontLeftPower); // actually Front left
            rightBack.setPower(-backRightPower); // actually Back Right
            rightFront.setPower(-frontRightPower); // actually Front Right

//            lift.run(gamepad2);

            if (gamepad2.a) {
                if (!aPressed) {
                    if (!armHigh) {
                        rightArmPos = 0.65;
                        leftArmPos = 0.4;
                        wristPosMaybe = 1;
                        armHigh = true;
                    } else {

                        armHigh = false;
                    }
                    aPressed = true;
                }
            } else {
                aPressed = false;
            }

            if (gamepad2.y) {
                if(!YPressed) {
                    if(!armUp){
                        leftArmPos = 0.318;
                        rightArmPos = 0.796;
                        wristPosMaybe = 0.893;
                        armUp=true;
                    } else {
                        leftArmPos = 1;
                        rightArmPos = 0;
                        wristPosMaybe = 0.58;
                        ;
                        armUp=false;
                    }
                    YPressed=true;
                }
            }else{
                YPressed = false;
            }

            if (gamepad2.left_stick_y < -0.3){
                wristPosMaybe -= 0.0025;
            }

            else if (gamepad2.left_stick_y > 0.3){
                wristPosMaybe += 0.0025;

            }


            if (gamepad2.right_stick_y < -0.3){
                leftArmPos -= 0.004;
                rightArmPos += 0.0067;
            }

            else if (gamepad2.right_stick_y > 0.3){
                leftArmPos += 0.004;
                rightArmPos -= 0.0067;
            }



            if (gamepad2.x) {
                if(!XPressed) {
                    if(!clawShut){
                        clawPosMaybe = 0;

                        clawShut=true;
                    } else {
                        clawPosMaybe = 0.4;

                        clawShut=false;
                    }
                    XPressed=true;
                }
            }else{
                XPressed = false;
            }

            leftArm.setPosition(leftArmPos);
            wrist.setPosition(wristPosMaybe);
            claw.setPosition(clawPosMaybe);
            rightArm.setPosition(rightArmPos);

            // Show the elapsed game time and wheel power.

            telemetry.addData("Arm Right Position", rightArm.getPosition());
            telemetry.addData("Arm Left Position", leftArm.getPosition());
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("lift level", lift.liftMotor.getCurrentPosition());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", backLeftPower, frontLeftPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backRightPower, frontRightPower);
            telemetry.update();

        }
    }
}