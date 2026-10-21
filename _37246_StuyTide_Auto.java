package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "_37246_StuyTide_Auto", group = "StarterBot")
public class _37246_StuyTide_Auto extends LinearOpMode {

    // Encoder counts per inch; assumes 537.7 counts per motor revolution and 4-inch wheels.
    private static final double DRIVE_COUNTS_PER_INCH = 537.7 / (4.0 * Math.PI);
    // Maximum drive power while moving to a target.
    private static final double DRIVE_POWER = 0.5;
    // Heading error, in degrees, allowed before a turn is complete.
    private static final double HEADING_TOLERANCE_DEGREES = 1.5;
    // Field heading at the start; assumes the robot starts facing away from the driver.
    private static final double STARTING_FIELD_HEADING_DEGREES = 90.0;
    // Launcher velocity while shooting.
    private static final double LAUNCHER_TARGET_VELOCITY = 1180;
    // Intake power while feeding balls into the launcher.
    private static final double SHOOT_FEED_POWER = 0.5;
    // Windmill power while feeding balls into the launcher.
    private static final double SHOOT_WINDMILL_POWER = 1.0;
    // Front-left drive motor.
    private DcMotor leftFrontDrive;
    // Rear-left drive motor.
    private DcMotor leftBackDrive;
    // Front-right drive motor.
    private DcMotor rightFrontDrive;
    // Rear-right drive motor.
    private DcMotor rightBackDrive;
    // Launcher motor.
    private DcMotorEx launcher;
    // Main intake motor.
    private DcMotor intake;
    // Left and right intake servos.
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    // Feeder servo that moves balls into the launcher.
    private CRServo windmillServo;
    // Heading sensor used for turns and field-relative movement.
    private IMU imu;

    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Drive movement is relative to the current position.");
        telemetry.addLine("Field heading: 0 right, positive counterclockwise.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        imu.resetYaw();

        // Add the autonomous route here using driveToPosition, shoot, and intakeFor.
        // shoot(2000);
        driveToPosition(5, 5, 90);
    }

    public void driveToPosition(
            // Relative horizontal movement in inches; right is positive.
            double xInches,
            // Relative vertical movement in inches; away from the driver is positive.
            double yInches,
            // Target field heading in degrees; right is 0 and counterclockwise is positive.
            double headingDegrees) {
        if (!Double.isFinite(xInches) || !Double.isFinite(yInches)
                || !Double.isFinite(headingDegrees)) {
            throw new IllegalArgumentException("Movement and heading must be finite numbers");
        }
        if (!opModeIsActive()) {
            return;
        }

        turnToHeading(headingDegrees);
        if (!opModeIsActive()) {
            return;
        }

        double heading = Math.toRadians(getFieldHeadingDegrees());
        double robotStrafe = xInches * Math.sin(heading) - yInches * Math.cos(heading);
        double robotForward = xInches * Math.cos(heading) + yInches * Math.sin(heading);

        int forwardTicks = (int) Math.round(robotForward * DRIVE_COUNTS_PER_INCH);
        int strafeTicks = (int) Math.round(robotStrafe * DRIVE_COUNTS_PER_INCH);
        int leftFrontTarget = leftFrontDrive.getCurrentPosition() + forwardTicks + strafeTicks;
        int rightFrontTarget = rightFrontDrive.getCurrentPosition() + forwardTicks - strafeTicks;
        int leftBackTarget = leftBackDrive.getCurrentPosition() + forwardTicks - strafeTicks;
        int rightBackTarget = rightBackDrive.getCurrentPosition() + forwardTicks + strafeTicks;

        int maxTicks = Math.max(
                Math.max(Math.abs(forwardTicks + strafeTicks), Math.abs(forwardTicks - strafeTicks)),
                1);
        leftFrontDrive.setTargetPosition(leftFrontTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);
        leftBackDrive.setTargetPosition(leftBackTarget);
        rightBackDrive.setTargetPosition(rightBackTarget);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(DRIVE_POWER * Math.abs(forwardTicks + strafeTicks) / maxTicks);
        rightFrontDrive.setPower(DRIVE_POWER * Math.abs(forwardTicks - strafeTicks) / maxTicks);
        leftBackDrive.setPower(DRIVE_POWER * Math.abs(forwardTicks - strafeTicks) / maxTicks);
        rightBackDrive.setPower(DRIVE_POWER * Math.abs(forwardTicks + strafeTicks) / maxTicks);

        try {
            while (opModeIsActive() && isDriveBusy()) {
                telemetry.addData("Move", "(%.1f, %.1f) in", xInches, yInches);
                telemetry.addData("Field heading", "%.1f", getFieldHeadingDegrees());
                telemetry.update();
                idle();
            }
        } finally {
            stopDrive();
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Runs the launcher and feeder continuously for the requested duration.
    public void shoot(
            // Shooting interval in milliseconds.
            long durationMs) {
        if (durationMs < 0) {
            throw new IllegalArgumentException("durationMs must not be negative");
        }
        if (!opModeIsActive()) {
            return;
        }

        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        try {
            setIntakePower(SHOOT_FEED_POWER);
            windmillServo.setPower(SHOOT_WINDMILL_POWER);
            sleep(durationMs);
        } finally {
            windmillServo.setPower(0);
            setIntakePower(0);
            launcher.setVelocity(0);
        }
    }

    public void intakeFor(
            // Intake power from 0 (off) to 1 (full intake).
            double power,
            // Duration to run the intake, in milliseconds.
            long durationMs) {
        if (!Double.isFinite(power) || power < 0.0 || power > 1.0) {
            throw new IllegalArgumentException("power must be between 0.0 and 1.0");
        }
        if (durationMs < 0) {
            throw new IllegalArgumentException("durationMs must not be negative");
        }
        if (!opModeIsActive()) {
            return;
        }

        setIntakePower(power);
        try {
            sleep(durationMs);
        } finally {
            setIntakePower(0);
        }
    }

    private void initializeHardware() {
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

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        setIntakePower(0);
        windmillServo.setPower(0);
        stopDrive();
    }

    private void turnToHeading(double targetHeadingDegrees) {
        while (opModeIsActive()) {
            double error = normalizeDegrees(targetHeadingDegrees - getFieldHeadingDegrees());
            if (Math.abs(error) <= HEADING_TOLERANCE_DEGREES) {
                break;
            }

            double rotationPower = Math.min(0.5, Math.max(0.12, Math.abs(error) * 0.01));
            setRobotDrivePower(0, 0, -Math.signum(error) * rotationPower);
            telemetry.addData("Target heading", "%.1f", targetHeadingDegrees);
            telemetry.addData("Field heading", "%.1f", getFieldHeadingDegrees());
            telemetry.update();
            idle();
        }
        stopDrive();
    }

    private double getFieldHeadingDegrees() {
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return normalizeDegrees(STARTING_FIELD_HEADING_DEGREES + imuHeading);
    }

    private static double normalizeDegrees(double degrees) {
        return ((degrees + 180) % 360 + 360) % 360 - 180;
    }

    private void setRobotDrivePower(double forward, double strafe, double rotate) {
        double leftFront = forward + strafe + rotate;
        double rightFront = forward - strafe - rotate;
        double leftBack = forward - strafe + rotate;
        double rightBack = forward + strafe - rotate;
        double max = Math.max(
                Math.max(Math.abs(leftFront), Math.abs(rightFront)),
                Math.max(Math.abs(leftBack), Math.abs(rightBack)));
        if (max > 1.0) {
            leftFront /= max;
            rightFront /= max;
            leftBack /= max;
            rightBack /= max;
        }
        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
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
}