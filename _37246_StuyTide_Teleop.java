package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-oriented TeleOp.
 *
 * HEADING CONVENTION (same as the Auto file):
 *   0 degrees   = field right (toward the driver's right hand)
 *   90 degrees  = field forward (away from the driver)
 *   Positive    = counterclockwise when viewed from above
 *
 * CONTROLS (gamepad1)
 *   Left stick   : move the robot across the field (up = away from driver)
 *   Right stick  : rotate the robot
 *   Triggers     : intake (right) / outtake (left)
 *   Right bumper : hold to shoot (spins the launcher up if needed, then
 *                  feeds automatically once it is at speed)
 *   Circle       : toggle launcher pre-spin. Turn it on while driving to the
 *                  goal so the launcher is already at speed when you shoot.
 *   Triangle     : re-zero heading. Point the robot's front at the driver's
 *                  right (START_HEADING_DEGREES), then press.
 */
@TeleOp(name = "_37246_StuyTide_Teleop", group = "StarterBot")
public class _37246_StuyTide_Teleop extends OpMode {

    // ---------------------------------------------------------------
    // Tunable constants
    // ---------------------------------------------------------------

    // Direction the robot's front faces when the heading is zeroed.
    // 0 = the intake points toward the driver's right.
    private static final double START_HEADING_DEGREES = 0.0;

    // Strafing is less efficient than driving forward on mecanum wheels,
    // so sideways commands are boosted by this factor.
    private static final double STRAFE_CORRECTION = 1.1;

    // Multiplies the right stick for turning. 1.0 = stick directly sets turn
    // power; higher values reach full turn speed with less stick travel
    // (1.5 = full turn speed at two-thirds stick). Raise it if turning still
    // feels slow; lower it if turning feels twitchy.
    private static final double TURN_GAIN = 1.5;

    private static final double LAUNCHER_TARGET_VELOCITY = 1180;
    private static final double LAUNCHER_MIN_VELOCITY = 1100;

    // Extra intake power added while a shot is being fed.
    private static final double SHOOT_INTAKE_BOOST = 0.5;

    // ---------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotorEx launcher;
    private DcMotor intake;

    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    private IMU imu;

    // ---------------------------------------------------------------
    // State (also shown in telemetry)
    // ---------------------------------------------------------------

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    private boolean launcherPreSpin = false;   // toggled with Circle
    private boolean launcherRunning = false;   // is the launcher being commanded to spin?
    private double launcherVelocity = 0;       // read once per loop

    // Previous button states, used to detect a fresh press (not a hold).
    private boolean previousTriangle = false;
    private boolean previousCircle = false;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        // Control Hub mounting: logo faces robot-left, USB ports face up.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        // The left motors are mounted mirrored from the right ones, so one
        // side must be reversed for positive power to mean "forward" on
        // all four wheels.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Open-loop drive: power maps straight to motor voltage. RUN_USING_ENCODER
        // would cap top speed at a fraction of the motor's maximum, and setting
        // it explicitly means TeleOp doesn't depend on the mode Auto left behind.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake mode: the robot stops quickly when the sticks are released.
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // Launcher: velocity control. Far below the target speed this outputs
        // full motor power, so spin-up is already as fast as the motor allows.
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        // Intake servo directions.
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Mode", "Field-Oriented");
        telemetry.addLine("Point the intake at the driver's right before Start.");
    }

    @Override
    public void start() {
        imu.resetYaw();
        previousTriangle = false;
        previousCircle = false;
        launcherPreSpin = false;
    }

    @Override
    public void loop() {
        // ---- Button presses (act once per press, not every loop) ----
        if (gamepad1.triangle && !previousTriangle) {
            imu.resetYaw();   // current facing becomes START_HEADING_DEGREES
        }
        previousTriangle = gamepad1.triangle;

        if (gamepad1.circle && !previousCircle) {
            launcherPreSpin = !launcherPreSpin;
        }
        previousCircle = gamepad1.circle;

        // ---- Driving ----
        // Read the heading once per loop and reuse it.
        double heading = getHeadingDegrees();

        // Stick up (negative Y on the gamepad) = away from the driver.
        double turn = Range.clip(gamepad1.right_stick_x * TURN_GAIN, -1.0, 1.0);
        driveFieldRelative(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                turn,
                heading);

        // ---- Launcher + intake ----
        boolean feeding = updateLauncher();

        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (feeding) {
            intakePower += SHOOT_INTAKE_BOOST;
        }
        intakePower = Range.clip(intakePower, -1.0, 1.0);

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        // ---- Telemetry ----
        String launcherState = !launcherRunning ? "OFF"
                : (launcherVelocity > LAUNCHER_MIN_VELOCITY ? "READY" : "SPINNING UP");
        telemetry.addData("Heading", "%.1f deg", heading);
        telemetry.addData("Front", "LF %.2f | RF %.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back", "LB %.2f | RB %.2f", leftBackPower, rightBackPower);
        telemetry.addData("Launcher", "%s (%.0f)", launcherState, launcherVelocity);
        telemetry.addData("Intake power", "%.2f", intakePower);
        telemetry.update();
    }

    @Override
    public void stop() {
        setMotorPowers(0, 0, 0, 0);
        launcher.setVelocity(0);
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
    }

    // ---------------------------------------------------------------
    // Driving
    // ---------------------------------------------------------------

    /**
     * Field-oriented drive.
     *
     * @param fieldX  desired movement toward field-right (-1 to 1)
     * @param fieldY  desired movement away from the driver (-1 to 1)
     * @param turn    rotation command; positive = clockwise (-1 to 1)
     * @param headingDegrees robot heading on the field (0 = right, CCW positive)
     *
     * The stick gives a direction in FIELD terms. The wheels need a direction
     * in ROBOT terms. If the robot is rotated by heading h, the field vector
     * is rotated by -h to express it in the robot's own frame:
     *
     *   robotForward = fieldX * cos(h) + fieldY * sin(h)
     *   robotRight   = fieldX * sin(h) - fieldY * cos(h)
     */
    private void driveFieldRelative(
            double fieldX, double fieldY, double turn, double headingDegrees) {

        double h = Math.toRadians(headingDegrees);
        double cos = Math.cos(h);
        double sin = Math.sin(h);

        double robotForward = fieldX * cos + fieldY * sin;
        double robotRight = (fieldX * sin - fieldY * cos) * STRAFE_CORRECTION;

        driveRobotRelative(robotForward, robotRight, turn);
    }

    /**
     * Standard mecanum mixing; all three inputs are robot-relative.
     * For example, forward + right makes the left-front and right-back wheels
     * spin forward while the other two cancel, sliding the robot diagonally.
     */
    private void driveRobotRelative(double forward, double right, double turn) {
        double lf = forward + right + turn;
        double rf = forward - right - turn;
        double lb = forward - right + turn;
        double rb = forward + right - turn;

        // If any wheel is asked for more than 100%, scale all four down by the
        // same amount. That keeps the direction of travel correct while still
        // running the fastest wheel at full power.
        double max = Math.max(
                Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        setMotorPowers(lf, rf, lb, rb);
    }

    private void setMotorPowers(double lf, double rf, double lb, double rb) {
        leftFrontPower = lf;
        rightFrontPower = rf;
        leftBackPower = lb;
        rightBackPower = rb;

        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }

    /** Robot heading on the field in degrees, range -180..180. */
    private double getHeadingDegrees() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return normalizeDegrees(START_HEADING_DEGREES + yaw);
    }

    /**
     * Wraps any angle into -180..180 so, for example, 190 becomes -170.
     * (Java's % keeps the sign of the input, hence the extra + 360.)
     */
    private static double normalizeDegrees(double degrees) {
        return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    // ---------------------------------------------------------------
    // Launcher
    // ---------------------------------------------------------------

    /**
     * Runs the launcher while pre-spin is toggled on or the right bumper is
     * held, and feeds the windmill only while the bumper is held AND the
     * launcher is up to speed.
     *
     * @return true while balls are being fed
     */
    private boolean updateLauncher() {
        boolean shootHeld = gamepad1.right_bumper;

        launcherRunning = launcherPreSpin || shootHeld;
        launcher.setVelocity(launcherRunning ? LAUNCHER_TARGET_VELOCITY : 0);

        launcherVelocity = launcher.getVelocity();   // read once, reused below

        boolean feeding = shootHeld && launcherVelocity > LAUNCHER_MIN_VELOCITY;
        windmillServo.setPower(feeding ? 1.0 : 0);
        return feeding;
    }
}