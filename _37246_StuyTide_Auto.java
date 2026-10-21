package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Autonomous with simple movement commands. All driving and turning runs at
 * full power.
 *
 * COORDINATES (same heading convention as the TeleOp file)
 *   Origin (0, 0) = where the robot starts.
 *   +X = field right, +Y = away from the driver, in inches.
 *   Heading: 0 = field right, 90 = away from driver, counterclockwise positive.
 *
 * MOVEMENT COMMANDS
 *   goTo(x, y, heading)               Go to an ABSOLUTE field spot, turning and
 *                                     driving at the same time.
 *   moveBy(dx, dy, heading)           Move RELATIVE to the current position, in
 *                                     FIELD directions (+X right, +Y away),
 *                                     turning and driving at the same time.
 *   driveRobot(forward, right)        Move relative to where the ROBOT is facing
 *                                     (forward = the intake side). No turning.
 *   turnTo(heading)                   Turn to an absolute field heading.
 *   turnBy(degrees)                   Turn relative to current heading (+ = CCW).
 *
 * MECHANISM COMMANDS
 *   startLauncher() / stopLauncher()  Spin the launcher up early (for example
 *                                     while driving) so shoot() has no delay.
 *   shoot(feedMs)                     Feed balls for feedMs once the launcher
 *                                     is at speed (spins it up if needed).
 *   startIntake(power) / stopIntake() Intake on/off without waiting, so the
 *                                     robot can move while it runs.
 *   waitMs(ms)                        Pause the route for a number of ms.
 *
 * NOTE: position is measured from the wheel encoders and the IMU (dead
 * reckoning), so it drifts a little with wheel slip. Keep routes short or
 * re-align against a wall.
 */
@Autonomous(name = "_37246_StuyTide_Auto", group = "StarterBot")
public class _37246_StuyTide_Auto extends LinearOpMode {

    // ---------------------------------------------------------------
    // Tunable constants
    // ---------------------------------------------------------------

    // Direction the robot's front faces at the start of the match.
    // 90 = facing away from the driver. 0 = facing the driver's right
    // (which is how the TeleOp assumes the robot is placed).
    private static final double START_HEADING_DEGREES = 0.0;

    // Encoder counts per inch of wheel travel (537.7 counts/rev, 4-inch wheels).
    // Measure a real 48-inch drive and adjust if the robot over/undershoots.
    private static final double COUNTS_PER_INCH = 537.7 / (4.0 * Math.PI);

    // Mecanum strafing travels less than the wheels turn; same value as TeleOp.
    // Increase if the robot strafes short, decrease if it strafes long.
    private static final double STRAFE_CORRECTION = 1.1;

    // How close counts as "arrived". Movement also ends if the robot passes
    // the target, since at full power it can skip right over a small window.
    private static final double POSITION_TOLERANCE_INCHES = 1.0;
    private static final double TURN_TOLERANCE_DEGREES = 1.5;

    // Safety timeouts so a stuck robot can't hang the OpMode.
    private static final double MOVE_TIMEOUT_SECONDS = 8.0;
    private static final double TURN_TIMEOUT_SECONDS = 4.0;

    private static final double LAUNCHER_TARGET_VELOCITY = 1180;
    private static final double LAUNCHER_MIN_VELOCITY = 1100;
    private static final long LAUNCHER_SPINUP_TIMEOUT_MS = 3000;
    private static final double SHOOT_FEED_POWER = 0.5;
    private static final double SHOOT_WINDMILL_POWER = 1.0;

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
    // State
    // ---------------------------------------------------------------

    // Tracked position on the field (inches, relative to the start).
    private double poseX = 0;
    private double poseY = 0;

    // Wheel encoder readings at the last odometry update.
    private int lastLeftFront;
    private int lastRightFront;
    private int lastLeftBack;
    private int lastRightBack;

    // True between startLauncher() and stopLauncher().
    private boolean launcherSpinning = false;

    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Origin = start position. +X right, +Y away from driver.");
        telemetry.addLine("Heading: 0 = right, 90 = away from driver, CCW positive.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // The robot's position and heading at Start become the reference.
        imu.resetYaw();
        poseX = 0;
        poseY = 0;
        syncEncoders();

        try {
            // ----------------------- YOUR ROUTE GOES HERE -----------------------
            // Examples:
            //   startLauncher();                // spin up early while driving...
            //   goTo(24, 0, 90);                // go to x=24, y=0, ending facing away from driver
            //   shoot(2000);                    // ...then shoot with no spin-up wait
            //   goTo(24, 24, 180);              // turns toward 180 while driving there
            //   moveBy(0, 12);                  // 12 in toward the far side of the field
            //   moveBy(-6, 0, 180);             // 6 in field-left, ending facing field-left
            //   driveRobot(12, 0);              // 12 in forward (the way the robot faces)
            //   driveRobot(0, -6);              // 6 in to the robot's left
            //   turnTo(180);                    // face field-left
            //   turnBy(-90);                    // turn 90 degrees clockwise
            //   waitMs(500);                    // pause for half a second
            //   startIntake(1.0);               // intake on while driving...
            //   driveRobot(24, 0);
            //   stopIntake();                   // ...then off
            
            
            // Works. Simplest auto
            // shoot(5000);
            // waitMs(500);
            // goTo(-28, -38, -90);
            // moveBy(-76, 0);
            // moveBy(0, -4);
            
            // shoot(5000);
            turnBy(-30);
            startIntake(1.0);
            moveBy(0, -35);
            turnBy(30);
            moveBy(10, 0);
            moveBy(0, -10);
            moveBy(0, 10);
            waitMs(1000);
            stopIntake();
            goTo(0, 10, 180);
            goTo(-130, 4, 180);
            shoot(5000);
            moveBy(10, 0);
            moveBy(0, -30);
            goTo(-104, -42, 90);
            
            // waitMs(500);
            // goTo(-28, -38, -90);
            // moveBy(-76, 0);
            // moveBy(0, -4);
            
            // --------------------------------------------------------------------

            updateOdometry();
            telemetry.addData("Final position", "(%.1f, %.1f) in", poseX, poseY);
            telemetry.addData("Final heading", "%.1f", getHeadingDegrees());
            telemetry.update();
        } finally {
            shutDown();   // everything off, even if the route throws an error
        }
    }

    // ===============================================================
    // Movement commands
    // ===============================================================

    /**
     * Drive to an absolute field position while rotating to the given heading.
     * Driving and turning happen at the same time, at full power. Each loop it
     * checks where the robot is and steers toward the target, so the path
     * stays straight even as the robot rotates.
     *
     * To spin in place, pass the position the robot is already at.
     *
     * @param xInches        target X, field right is positive
     * @param yInches        target Y, away from the driver is positive
     * @param headingDegrees final heading (0 = right, 90 = away, CCW positive)
     */
    public void goTo(double xInches, double yInches, double headingDegrees) {
        requireFinite(xInches, yInches, headingDegrees);
        if (!opModeIsActive()) {
            return;
        }

        // Remember which way the target was when we started. Later we compare
        // against it to notice if the robot has gone past the target.
        double heading = updateOdometry();
        double startDx = xInches - poseX;
        double startDy = yInches - poseY;
        double startHeadingError = normalizeDegrees(headingDegrees - heading);

        // Once the position or heading is reached (or passed), it stays done,
        // so a full-power overshoot can't make the robot hunt back and forth.
        boolean positionDone = false;
        boolean headingDone = false;

        ElapsedTime timer = new ElapsedTime();
        try {
            while (opModeIsActive() && timer.seconds() < MOVE_TIMEOUT_SECONDS) {
                heading = updateOdometry();

                double dx = xInches - poseX;
                double dy = yInches - poseY;
                double distance = Math.hypot(dx, dy);
                double headingError = normalizeDegrees(headingDegrees - heading);

                // "Passed the target" = the remaining vector now points the
                // opposite way from where we started (dot product <= 0).
                if (distance <= POSITION_TOLERANCE_INCHES
                        || dx * startDx + dy * startDy <= 0) {
                    positionDone = true;
                }
                // Same idea for heading: the error changed sign.
                if (Math.abs(headingError) <= TURN_TOLERANCE_DEGREES
                        || headingError * startHeadingError <= 0) {
                    headingDone = true;
                }
                if (positionDone && headingDone) {
                    break;
                }

                // Full power straight at the target, as a FIELD direction.
                // (distance > 0 here, since positionDone is false.)
                double fieldX = 0;
                double fieldY = 0;
                if (!positionDone) {
                    fieldX = dx / distance;
                    fieldY = dy / distance;
                }

                // Full power rotation. Positive error = target is counterclockwise
                // of us, and positive turn power = clockwise, so negate.
                double turn = headingDone ? 0 : -Math.signum(headingError);

                // Same field-oriented math as the TeleOp.
                driveFieldRelative(fieldX, fieldY, turn, heading);

                telemetry.addData("Target", "(%.1f, %.1f) @ %.1f", xInches, yInches, headingDegrees);
                telemetry.addData("Position", "(%.1f, %.1f)", poseX, poseY);
                telemetry.addData("Heading", "%.1f", heading);
                telemetry.update();
                idle();
            }
        } finally {
            stopDrive();
        }
    }

    /**
     * Move relative to the CURRENT position, in field directions
     * (+X = field right, +Y = away from driver), rotating to the given heading
     * while driving (like goTo).
     */
    public void moveBy(double dxInches, double dyInches, double headingDegrees) {
        requireFinite(dxInches, dyInches, headingDegrees);
        if (!opModeIsActive()) {
            return;
        }
        updateOdometry();
        goTo(poseX + dxInches, poseY + dyInches, headingDegrees);
    }

    /** moveBy, but keeps the robot's current heading. */
    public void moveBy(double dxInches, double dyInches) {
        moveBy(dxInches, dyInches, getHeadingDegrees());
    }

    /**
     * Move relative to the direction the robot is currently facing, without
     * turning. This is the most precise move: the motor controllers slow each
     * wheel down as it reaches its target.
     *
     * @param forwardInches inches toward the robot's front (negative = backward)
     * @param rightInches   inches toward the robot's right (negative = left)
     */
    public void driveRobot(double forwardInches, double rightInches) {
        requireFinite(forwardInches, rightInches);
        if (!opModeIsActive()) {
            return;
        }
        updateOdometry();

        double strafe = rightInches * STRAFE_CORRECTION;

        // Distance each wheel must travel (inches of wheel rotation).
        // Forward moves all four the same way; strafing moves the two
        // diagonal pairs in opposite directions.
        double lf = forwardInches + strafe;
        double rf = forwardInches - strafe;
        double lb = forwardInches - strafe;
        double rb = forwardInches + strafe;

        double maxTravel = Math.max(
                Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (maxTravel < 0.01) {
            return;   // nothing to do
        }

        // Each wheel's target = where it is now + the distance it must travel.
        leftFrontDrive.setTargetPosition(
                leftFrontDrive.getCurrentPosition() + toCounts(lf));
        rightFrontDrive.setTargetPosition(
                rightFrontDrive.getCurrentPosition() + toCounts(rf));
        leftBackDrive.setTargetPosition(
                leftBackDrive.getCurrentPosition() + toCounts(lb));
        rightBackDrive.setTargetPosition(
                rightBackDrive.getCurrentPosition() + toCounts(rb));
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // The wheel that travels the farthest runs at full power; the others
        // are scaled to match so all wheels arrive together and the robot
        // travels in a straight line. RUN_TO_POSITION slows each wheel as it
        // reaches its target.
        leftFrontDrive.setPower(Math.abs(lf) / maxTravel);
        rightFrontDrive.setPower(Math.abs(rf) / maxTravel);
        leftBackDrive.setPower(Math.abs(lb) / maxTravel);
        rightBackDrive.setPower(Math.abs(rb) / maxTravel);

        ElapsedTime timer = new ElapsedTime();
        try {
            while (opModeIsActive() && isDriveBusy()
                    && timer.seconds() < MOVE_TIMEOUT_SECONDS) {
                double heading = updateOdometry();
                telemetry.addData("Moving", "forward %.1f, right %.1f in",
                        forwardInches, rightInches);
                telemetry.addData("Position", "(%.1f, %.1f)", poseX, poseY);
                telemetry.addData("Heading", "%.1f", heading);
                telemetry.update();
                idle();
            }
        } finally {
            stopDrive();
            setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        updateOdometry();
    }

    /**
     * Turn in place, at full power, to an absolute field heading.
     *
     * @param headingDegrees 0 = field right, 90 = away from driver, CCW positive
     */
    public void turnTo(double headingDegrees) {
        requireFinite(headingDegrees);
        if (!opModeIsActive()) {
            return;
        }

        double startError = normalizeDegrees(headingDegrees - updateOdometry());

        ElapsedTime timer = new ElapsedTime();
        try {
            while (opModeIsActive() && timer.seconds() < TURN_TIMEOUT_SECONDS) {
                double heading = updateOdometry();
                double error = normalizeDegrees(headingDegrees - heading);

                // Done when close enough, or when we have turned past the target.
                if (Math.abs(error) <= TURN_TOLERANCE_DEGREES || error * startError <= 0) {
                    break;
                }

                // Positive error = target is counterclockwise of us.
                // Positive wheel rotation power = clockwise, so negate.
                setDrivePowers(0, 0, -Math.signum(error));

                telemetry.addData("Turning to", "%.1f", headingDegrees);
                telemetry.addData("Heading", "%.1f", heading);
                telemetry.update();
                idle();
            }
        } finally {
            stopDrive();
        }
    }

    /** Turn relative to the current heading. Positive = counterclockwise. */
    public void turnBy(double degrees) {
        requireFinite(degrees);
        turnTo(getHeadingDegrees() + degrees);
    }

    // ===============================================================
    // Mechanism and timing commands
    // ===============================================================

    /**
     * Start spinning the launcher and return immediately. Call this early (for
     * example before driving to the shooting spot) so the launcher is already
     * at speed when shoot() runs. It keeps spinning until stopLauncher().
     */
    public void startLauncher() {
        if (!opModeIsActive()) {
            return;
        }
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        launcherSpinning = true;
    }

    /** Stop the launcher started by startLauncher(). */
    public void stopLauncher() {
        launcher.setVelocity(0);
        launcherSpinning = false;
    }

    /**
     * Wait for the launcher to reach speed, then run the feeder for the given
     * time. If startLauncher() was called first, the launcher is left spinning
     * afterwards (so back-to-back shots have no spin-up wait); otherwise it is
     * stopped when the shot ends. Note this turns the intake off afterwards.
     *
     * @param feedMs how long to feed balls once the launcher is up to speed
     */
    public void shoot(long feedMs) {
        if (feedMs < 0) {
            throw new IllegalArgumentException("feedMs must not be negative");
        }
        if (!opModeIsActive()) {
            return;
        }

        boolean keepSpinning = launcherSpinning;
        startLauncher();
        try {
            // Wait (up to a timeout) until the launcher is fast enough to shoot.
            ElapsedTime spinUp = new ElapsedTime();
            while (opModeIsActive()
                    && launcher.getVelocity() < LAUNCHER_MIN_VELOCITY
                    && spinUp.milliseconds() < LAUNCHER_SPINUP_TIMEOUT_MS) {
                idle();
            }

            // Same feeding as the TeleOp: windmill pushes balls into the launcher
            // while the intake helps move them along.
            setIntakePower(SHOOT_FEED_POWER);
            windmillServo.setPower(SHOOT_WINDMILL_POWER);
            waitMs(feedMs);
        } finally {
            windmillServo.setPower(0);
            setIntakePower(0);
            if (!keepSpinning) {
                stopLauncher();
            }
        }
    }

    /**
     * Start the intake and return immediately, so the robot can keep moving.
     * Call stopIntake() when you're done.
     *
     * @param power -1 (outtake) to 1 (full intake)
     */
    public void startIntake(double power) {
        if (!Double.isFinite(power) || power < -1.0 || power > 1.0) {
            throw new IllegalArgumentException("power must be between -1.0 and 1.0");
        }
        if (!opModeIsActive()) {
            return;
        }
        setIntakePower(power);
    }

    /** Stop the intake started by startIntake(). */
    public void stopIntake() {
        setIntakePower(0);
    }

    /**
     * Pause the route for a number of milliseconds. Stops early if the
     * OpMode is stopped. Anything already running (like the intake) keeps going.
     */
    public void waitMs(long ms) {
        if (ms < 0) {
            throw new IllegalArgumentException("ms must not be negative");
        }
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) {
            idle();
        }
    }

    // ===============================================================
    // Internals
    // ===============================================================

    private void initializeHardware() {
        // Read all hub sensor data (encoders, velocities) in one bulk transfer
        // instead of one transfer per read. This makes each loop much faster,
        // which matters because goTo checks position every loop. AUTO mode
        // refreshes the data on its own, so nothing else is needed.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        imu = hardwareMap.get(IMU.class, "imu");

        // Control Hub mounting: logo faces robot-left, USB ports face up.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // The left motors are mounted mirrored from the right ones, so one
        // side must be reversed for positive power to mean "forward" on
        // all four wheels (this also flips their encoder readings to match).
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Zero the encoders, then run open-loop. Encoders still count in this
        // mode (needed for odometry); RUN_USING_ENCODER would only cap top speed.
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        shutDown();
        syncEncoders();
    }

    /** Turn everything off. */
    private void shutDown() {
        stopDrive();
        setIntakePower(0);
        windmillServo.setPower(0);
        launcher.setVelocity(0);
        launcherSpinning = false;
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

    private static int toCounts(double inches) {
        return (int) Math.round(inches * COUNTS_PER_INCH);
    }

    /** Remember the current wheel encoder readings as the odometry baseline. */
    private void syncEncoders() {
        lastLeftFront = leftFrontDrive.getCurrentPosition();
        lastRightFront = rightFrontDrive.getCurrentPosition();
        lastLeftBack = leftBackDrive.getCurrentPosition();
        lastRightBack = rightBackDrive.getCurrentPosition();
    }

    /**
     * Updates poseX / poseY from how far each wheel has turned since the last
     * call. The wheel movement is converted to robot-relative forward/right
     * (the reverse of the mecanum mixing), then rotated into field
     * coordinates using the IMU heading.
     *
     * @return the heading (degrees) used, so callers can reuse it
     */
    private double updateOdometry() {
        int lf = leftFrontDrive.getCurrentPosition();
        int rf = rightFrontDrive.getCurrentPosition();
        int lb = leftBackDrive.getCurrentPosition();
        int rb = rightBackDrive.getCurrentPosition();

        // Inches each wheel has turned since the last update.
        double dLf = (lf - lastLeftFront) / COUNTS_PER_INCH;
        double dRf = (rf - lastRightFront) / COUNTS_PER_INCH;
        double dLb = (lb - lastLeftBack) / COUNTS_PER_INCH;
        double dRb = (rb - lastRightBack) / COUNTS_PER_INCH;

        lastLeftFront = lf;
        lastRightFront = rf;
        lastLeftBack = lb;
        lastRightBack = rb;

        // Undo the mecanum mixing: the average of all four wheels is forward
        // movement, and the diagonal difference is sideways movement.
        // Spinning in place cancels out of both of these.
        double forward = (dLf + dRf + dLb + dRb) / 4.0;
        double right = (dLf - dRf - dLb + dRb) / 4.0 / STRAFE_CORRECTION;

        // Rotate that robot-relative movement into field coordinates.
        double heading = getHeadingDegrees();
        double h = Math.toRadians(heading);
        poseX += forward * Math.cos(h) + right * Math.sin(h);
        poseY += forward * Math.sin(h) - right * Math.cos(h);
        return heading;
    }

    /**
     * Field-oriented drive (same math as the TeleOp): converts a movement
     * direction in FIELD terms into ROBOT terms using the current heading.
     *
     * @param fieldX field-right power
     * @param fieldY away-from-driver power
     * @param turn   clockwise-positive rotation power
     */
    private void driveFieldRelative(
            double fieldX, double fieldY, double turn, double headingDegrees) {
        double h = Math.toRadians(headingDegrees);
        double robotForward = fieldX * Math.cos(h) + fieldY * Math.sin(h);
        double robotRight = (fieldX * Math.sin(h) - fieldY * Math.cos(h)) * STRAFE_CORRECTION;
        setDrivePowers(robotForward, robotRight, turn);
    }

    /** Standard mecanum mixing; inputs are robot-relative, turn is clockwise-positive. */
    private void setDrivePowers(double forward, double right, double turn) {
        double lf = forward + right + turn;
        double rf = forward - right - turn;
        double lb = forward - right + turn;
        double rb = forward + right - turn;

        // If any wheel is asked for more than 100%, scale all four down by the
        // same amount so the direction of travel stays correct.
        double max = Math.max(
                Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    private void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private boolean isDriveBusy() {
        return leftFrontDrive.isBusy() || rightFrontDrive.isBusy()
                || leftBackDrive.isBusy() || rightBackDrive.isBusy();
    }

    private void setIntakePower(double power) {
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    private static void requireFinite(double... values) {
        for (double v : values) {
            if (!Double.isFinite(v)) {
                throw new IllegalArgumentException("Values must be finite numbers");
            }
        }
    }
}