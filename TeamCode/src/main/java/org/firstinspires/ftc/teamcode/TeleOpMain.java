package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleOpMain  extends OpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo left_Feeder = null;
    private CRServo right_Feeder = null;

    ElapsedTime feederTimer = new ElapsedTime();
    
    /* This enum declares the different states the robot
     can be in during the launching function */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;


    //Code to ran ONCE when the driver hits INIT
    @Override
    public void init() {
        launchState = LaunchState.IDLE;


        // Mapping hardware varibales to corresponding names assigned in robot configuration
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        left_Feeder = hardwareMap.get(CRServo.class, "left_feeder");
        right_Feeder = hardwareMap.get(CRServo.class, "right_feeder");


        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set the motor to run through the RUN_USING_ENCODER MODE
        // Tell the motor to do its best to run a target velocity.
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
           Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
           slow down much faster when it is coasting. This creates a much more controllable
           drivetrain. As the robot stops much quicker.
         */
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
           set Feeders to an initial value to initialize the servo controller
         */
        left_Feeder.setPower(STOP_SPEED);
        right_Feeder.setPower(STOP_SPEED);


        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
           Much like our drivetrain motors, we set the left feeder servo to reverse so that they
           both work to feed the ball into the robot.
         */
        left_Feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /* Call arcadeDrive Function
        * Move the left joystick forward to drive forward.
        * Moving the left joystick back does nothing.
        * Move the right joystick left and right to turn the robot.
        */
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        }

        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad1.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("targetSpeed", 1125.0);
        telemetry.addData("motorSpeed", launcher.getVelocity());

    }
    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;
        /*
         * Send calculated power to wheels
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                left_Feeder.setPower(FULL_SPEED);
                right_Feeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    left_Feeder.setPower(STOP_SPEED);
                    right_Feeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}
