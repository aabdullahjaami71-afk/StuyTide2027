
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "_37246_StuyTide_Teleop", group = "StarterBot")
public class _37246_StuyTide_Teleop extends OpMode {

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

    public final int LAUNCHER_TARGET_VELOCITY = 1180;
    public final int LAUNCHER_MIN_VELOCITY = 1100;

    public final double STRAFE_CORRECTION = 1.1;

    // Field heading offset:
    // Robot front (intake) initially points to the driver's right.
    // Field forward points away from the driver.
    private static final double FIELD_HEADING_OFFSET = -90.0;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double intakePower;

    private boolean previousTriangle = false;

    @Override
    public void init() {

        leftFrontDrive = hardwareMap.get(
                DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(
                DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(
                DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(
                DcMotor.class, "right_back_drive");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        windmillServo = hardwareMap.get(
                CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(
                CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(
                CRServo.class, "right_intake_servo");

        // Control Hub mounting:
        // Logo faces robot-left; USB ports face upward.
        // This corresponds to the logo facing away from the
        // driver when the robot's intake faces the driver's right.
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(imuParameters);
        imu.resetYaw();

        // Drive motor directions.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake mode.
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // Launcher configuration.
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5)
        );

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        // Intake servo directions.
        rightIntakeServo.setDirection(
                DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(
                DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Mode", "Field-Oriented");
        telemetry.addData("Initial Field Offset", FIELD_HEADING_OFFSET);
    }

    @Override
    public void init_loop() {
        // Nothing required here.
    }

    @Override
    public void start() {
        // Establish the initial heading reference at match start.
        imu.resetYaw();
        previousTriangle = false;
    }

    @Override
    public void loop() {

        // Triangle resets heading once per press.
        if (gamepad1.triangle && !previousTriangle) {
            imu.resetYaw();
        }
        previousTriangle = gamepad1.triangle;

        // FIELD-ORIENTED DRIVE IS ALWAYS ACTIVE.
        // Up = away from driver.
        // Down = toward driver.
        // Left/right = field-relative strafing.
        // Right stick X = robot rotation.
        mecanumDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // Intake trigger control.
        intakePower =
                gamepad1.right_trigger - gamepad1.left_trigger;

        // Launcher and windmill control.
        launch();

        // Keep motor and servo powers within their valid range.
        double safeIntakePower = Math.max(
                -1.0, Math.min(1.0, intakePower));

        intake.setPower(safeIntakePower);
        leftIntakeServo.setPower(safeIntakePower);
        rightIntakeServo.setPower(safeIntakePower);

        // Telemetry.
        double yaw = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

        double fieldHeading = normalizeDegrees(
                yaw + FIELD_HEADING_OFFSET);

        telemetry.addData("Drive Mode", "FIELD-ORIENTED");
        telemetry.addData("IMU Yaw", "%.1f", yaw);
        telemetry.addData("Field Heading", "%.1f", fieldHeading);

        telemetry.addData(
                "Front Motors",
                "LF %.2f | RF %.2f",
                leftFrontPower, rightFrontPower
        );

        telemetry.addData(
                "Back Motors",
                "LB %.2f | RB %.2f",
                leftBackPower, rightBackPower
        );

        telemetry.addData(
                "Triggers",
                "Left %.2f | Right %.2f",
                gamepad1.left_trigger,
                gamepad1.right_trigger
        );

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop the drivetrain and mechanisms when OpMode ends.
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        launcher.setVelocity(0);
        intake.setPower(0);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
    }

    /**
     * Field-oriented mecanum drive.
     *
     * forward: positive means away from the driver.
     * strafe:  positive means field-right.
     * rotate:  right-stick X rotation command.
     *
     * The field-relative translation vector is transformed into
     * robot-relative coordinates using the IMU heading.
     */
    private void mecanumDrive(
            double forward,
            double strafe,
            double rotate) {

        double yaw = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

        // Convert the robot's measured heading to the field heading.
        double heading = Math.toRadians(
                normalizeDegrees(yaw + FIELD_HEADING_OFFSET));

        // Field-relative -> robot-relative coordinate transform.
        double robotStrafe =
                strafe * Math.cos(heading)
                + forward * Math.sin(heading);

        double robotForward =
                -strafe * Math.sin(heading)
                + forward * Math.cos(heading);

        // Compensate for mecanum strafing inefficiency.
        robotStrafe *= STRAFE_CORRECTION;

        // Standard mecanum wheel mixing.
        leftFrontPower =
                robotForward + robotStrafe + rotate;

        rightFrontPower =
                robotForward - robotStrafe - rotate;

        leftBackPower =
                robotForward - robotStrafe + rotate;

        rightBackPower =
                robotForward + robotStrafe - rotate;

        // Normalize together to preserve the movement direction.
        double max = Math.max(
                Math.max(
                        Math.abs(leftFrontPower),
                        Math.abs(rightFrontPower)),
                Math.max(
                        Math.abs(leftBackPower),
                        Math.abs(rightBackPower))
        );

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private double normalizeDegrees(double degrees) {
        return ((degrees + 180.0) % 360.0 + 360.0) % 360.0
                - 180.0;
    }

    private void launch() {

        if (gamepad1.right_bumper) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (gamepad1.right_bumper
                && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {

            windmillServo.setPower(1.0);
            intakePower += 0.5;

        } else {
            windmillServo.setPower(0);
        }
    }
}
