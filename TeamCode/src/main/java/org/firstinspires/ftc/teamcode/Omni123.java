package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.mechanism.Lift;

@TeleOp(name="Omni Drive Simple", group="Linear Opmode")

public class Omni123 extends LinearOpMode {
    Lift lift = new Lift();


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;

    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    //    private DcMotor slide = null;
    double wristPos;
    private Servo wrist;
    private Servo armLeft;
    private Servo armRight;
    boolean bWasDown = false;
    double armLeftPos;
    double armRightPos;
    double clawPos;
    boolean YPressed = false;
    boolean XPressed = false;
    boolean armUp = false;
    boolean slow = false;
    boolean clawShut = false;
    boolean BPressed = false;

    private Servo claw;
    boolean xWasDown = false;

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRight");
        armRight = hardwareMap.get(Servo.class, "ArmRight");
        armLeft = hardwareMap.get(Servo.class, "ArmLeft");
        claw = hardwareMap.get(Servo.class, "Claw");
        wrist = hardwareMap.get(Servo.class, "Wrist");
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
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        slide.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        armRightPos = 1;
        armLeftPos = 0.0;
        clawPos = 0.8;
        wristPos = 0;


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
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

//            lift.run(gamepad2);

            if (gamepad2.y) {
                if(!YPressed) {
                    if(!armUp){
                        armRightPos = 0.60;
                        armLeftPos = 0.40;
                        wristPos = 0.6;
                        armUp=true;
                    } else {
                        armRightPos = 1;
                        armLeftPos = 0;
                        wristPos = 0;
                        ;
                        armUp=false;
                    }
                    YPressed=true;
                }
            }else{
                YPressed = false;
            }

            if (gamepad2.left_stick_y < -0.3){
                armRightPos -= 0.0025;
                armLeftPos +=0.0025;
                //parallel wrist
                wristPos += 0.0025;
            }

            else if (gamepad2.left_stick_y > 0.3){
                armRightPos += 0.0025;
                armLeftPos -=0.0025;
                //parallel wrist
                wristPos -= 0.0025;
            }


            if (gamepad2.right_stick_y < -0.3){
                wristPos -= 0.0025;
            }

            else if (gamepad2.right_stick_y > 0.3){
                wristPos += 0.0025;
            }



            if (gamepad2.x) {
                if(!XPressed) {
                    if(!clawShut){
                        clawPos = 0.5;
                        clawShut=true;
                    }else {
                        clawPos=0.8;
                        clawShut=false;
                    }
                    XPressed=true;
                }
            }else{
                XPressed = false;
            }

            armRight.setPosition(armRightPos);
            armLeft.setPosition(armLeftPos);
            claw.setPosition(clawPos);
            wrist.setPosition(wristPos);
//                armLeft.setPosition(armPos);
//                armRight.setPosition(armPos);

            // Show the elapsed game time and wheel power.

            telemetry.addData("Arm Right Position", armRight.getPosition());
            telemetry.addData("Arm Left Position", armLeft.getPosition());
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("lift level", lift.liftMotor.getCurrentPosition());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

        }
    }
}