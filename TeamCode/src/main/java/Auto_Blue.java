import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "Blue_Auto")
public class Auto_Blue extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor shooterLeft, shooterRight;
    DcMotor intake; // intake motor
    CRServo sorter;
    CRServo kickerLeft, kickerRight;

    // Constants
    static final double WHEEL_DIAMETER_MM = 104.0;
    static final double MM_PER_INCH = 25.4;
    static final double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * Math.PI;
    static final int TICKS_PER_REV = 560; // Change if your encoder spec is different

    static final double STRAFE_MULTIPLIER = 1.15;
    static final double ROBOT_TRACK_DIAMETER_MM = 330.0;

    // Shooter settings
    static final double SHOOTER_POWER = 0.9;
    static final long TOTAL_SHOOT_SEQUENCE_MS = 14000L; // total shooter sequence = 14s
    static final long INITIAL_DELAY_MS = 2000L;         // 2s delay before phase1 kicker/sorter
    static final long PHASE_MS = 5000L;                 // each kicker phase duration (5s)
    static final long GAP_MS = TOTAL_SHOOT_SEQUENCE_MS - INITIAL_DELAY_MS - PHASE_MS - PHASE_MS; // 2000ms gap

    // Combined maximum allowed absolute power for both shooters together
    static final double MAX_COMBINED_SHOOTER_POWER = 0.65;

    // Intake power (positive = intake in)
    static final double INTAKE_POWER = 0.75;

    // Kicker
    static final double KICKER_POWER = 1.0;

    // Sorter (CRServo) single power constant; phase2 will use negative to run opposite direction
    static final double SORTER_POWER = 1.0;

    @Override
    public void runOpMode() {
        // Hardware mapping
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        // Shooter motors (must exist in robot config)
        shooterLeft = hardwareMap.dcMotor.get("shooterLeft");
        shooterRight = hardwareMap.dcMotor.get("shooterRight");

        // Intake motor (must exist in robot config)
        intake = hardwareMap.dcMotor.get("intake");

        // Sorter CRServo (optional - guarded by try/catch when used)
        try {
            sorter = hardwareMap.get(CRServo.class, "sorter");
            sorter.setPower(0.0);
        } catch (Exception ignored) {
            sorter = null;
        }

        // Kicker CRServos (optional)
        try {
            kickerLeft = hardwareMap.get(CRServo.class, "kickerLeft");
            kickerLeft.setPower(0.0);
        } catch (Exception ignored) {
            kickerLeft = null;
        }
        try {
            kickerRight = hardwareMap.get(CRServo.class, "kickerRight");
            kickerRight.setPower(0.0);
        } catch (Exception ignored) {
            kickerRight = null;
        }

        // Set direction if needed (match your wiring)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Shooter directions: adjust if your wiring makes them spin incorrectly
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);

        // Intake direction: positive power = IN (adjust if your robot is inverted)
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setPower(0);

        // Reset encoders for drivetrain
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Use RUN_USING_ENCODER so getCurrentPosition() is meaningful and we can set relative targets
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Shooters run without encoders for simple on/off control
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        // 1) Move forward 36 inches at 3/4 power
        moveForwardInches(36, 0.75);

        // 2) Turn around (approx)
        turnLeftDegrees(370, 1);

        // 3) Shooting sequence (20s total)
        // Start shooter immediately and keep it running for the whole sequence.
        // After INITIAL_DELAY_MS start kickerLeft + sorter (SORTER_POWER) for PHASE_MS,
        // then gap for GAP_MS, then run kickerRight + sorter opposite (-SORTER_POWER) for PHASE_MS.
        setShooterPowers(SHOOTER_POWER, SHOOTER_POWER); // shooter starts right away

        // initial delay while shooter runs
        waitMs(INITIAL_DELAY_MS);
        if (!opModeIsActive()) {
            setShooterPowers(0.0, 0.0);
            return;
        }

        // Phase 1: kickerLeft + sorter (SORTER_POWER)
        if (kickerLeft != null) {
            try { kickerLeft.setPower(KICKER_POWER); } catch (Exception ignored) {}
        }
        if (sorter != null) {
            try { sorter.setPower(-SORTER_POWER); } catch (Exception ignored) {}
        }
        waitMs(PHASE_MS);

        // Stop phase1 kicker and sorter but keep shooter running
        if (kickerLeft != null) {
            try { kickerLeft.setPower(0.0); } catch (Exception ignored) {}
        }
        if (sorter != null) {
            try { sorter.setPower(0.0); } catch (Exception ignored) {}
        }
        if (!opModeIsActive()) {
            setShooterPowers(0.0, 0.0);
            return;
        }

        // Gap/stop period (shooter still running)
        waitMs(GAP_MS);
        if (!opModeIsActive()) {
            setShooterPowers(0.0, 0.0);
            return;
        }

        // Phase 2: flip shooter direction and run kickerRight + sorter opposite direction
        setShooterPowers(SHOOTER_POWER, SHOOTER_POWER); // flip shooter direction without stopping
        if (kickerRight != null) {
            try { kickerRight.setPower(-KICKER_POWER); } catch (Exception ignored) {}
        }
        if (sorter != null) {
            try { sorter.setPower(SORTER_POWER); } catch (Exception ignored) {}
        }
        waitMs(PHASE_MS);

        // stop phase2 kicker, sorter, and shooter
        if (kickerRight != null) {
            try { kickerRight.setPower(0.0); } catch (Exception ignored) {}
        }
        if (sorter != null) {
            try { sorter.setPower(0.0); } catch (Exception ignored) {}
        }
        setShooterPowers(0.0, 0.0);

        // Continue to step 4
        turnLeftDegrees(180, 1);

        // 5) Move forward 20 inches while running the intake
        intake.setPower(INTAKE_POWER);
        moveForwardInches(20, 0.35);
        waitMs(2000); // run intake for 2s after moving
        intake.setPower(0);
    }

    // Move forward by inches (relative targets)
    public void moveForwardInches(double inches, double power) {
        int ticks = inchesToTicks(inches);

        int flTarget = frontLeft.getCurrentPosition() + ticks;
        int frTarget = frontRight.getCurrentPosition() + ticks;
        int blTarget = backLeft.getCurrentPosition() + ticks;
        int brTarget = backRight.getCurrentPosition() + ticks;

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(power));
        frontRight.setPower(Math.abs(power));
        backLeft.setPower(Math.abs(power));
        backRight.setPower(Math.abs(power));

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            idle();
        }

        stopAndReturnToEncoderMode();
    }

    // Strafe left by inches (mecanum)
    public void strafeLeftInches(double inches, double power) {
        int ticks = (int)Math.round(inchesToTicks(inches) * STRAFE_MULTIPLIER);

        int flTarget = frontLeft.getCurrentPosition() - ticks; // backward
        int frTarget = frontRight.getCurrentPosition() + ticks; // forward
        int blTarget = backLeft.getCurrentPosition() + ticks; // forward
        int brTarget = backRight.getCurrentPosition() - ticks; // backward

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(power));
        frontRight.setPower(Math.abs(power));
        backLeft.setPower(Math.abs(power));
        backRight.setPower(Math.abs(power));

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            idle();
        }

        stopAndReturnToEncoderMode();
    }

    // Strafe right by inches (mecanum). Mirror of strafeLeftInches.
    public void strafeRightInches(double inches, double power) {
        int ticks = (int)Math.round(inchesToTicks(inches) * STRAFE_MULTIPLIER);

        int flTarget = frontLeft.getCurrentPosition() + ticks; // forward
        int frTarget = frontRight.getCurrentPosition() - ticks; // backward
        int blTarget = backLeft.getCurrentPosition() - ticks; // backward
        int brTarget = backRight.getCurrentPosition() + ticks; // forward

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(power));
        frontRight.setPower(Math.abs(power));
        backLeft.setPower(Math.abs(power));
        backRight.setPower(Math.abs(power));

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            idle();
        }

        stopAndReturnToEncoderMode();
    }

    // Turn left by degrees (positive = left)
    public void turnLeftDegrees(double degrees, double power) {
        double circumference = Math.PI * ROBOT_TRACK_DIAMETER_MM;
        double mm = (degrees / 360.0) * circumference;
        int ticks = inchesToTicks(mm / MM_PER_INCH); // convert mm -> inches -> ticks

        int flTarget = frontLeft.getCurrentPosition() - ticks;
        int frTarget = frontRight.getCurrentPosition() + ticks;
        int blTarget = backLeft.getCurrentPosition() - ticks;
        int brTarget = backRight.getCurrentPosition() + ticks;

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(power));
        frontRight.setPower(Math.abs(power));
        backLeft.setPower(Math.abs(power));
        backRight.setPower(Math.abs(power));

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            idle();
        }

        stopAndReturnToEncoderMode();
    }

    // Helper wait with opModeIsActive check
    private void waitMs(long ms) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < ms) {
            idle();
        }
    }

    // Scale requested shooter powers so |left|+|right| <= MAX_COMBINED_SHOOTER_POWER
    private void setShooterPowers(double leftRequested, double rightRequested) {
        double sum = Math.abs(leftRequested) + Math.abs(rightRequested);
        double scale = 1.0;
        if (sum > MAX_COMBINED_SHOOTER_POWER && sum > 0.0) {
            scale = MAX_COMBINED_SHOOTER_POWER / sum;
        }
        shooterLeft.setPower(leftRequested * scale);
        shooterRight.setPower(rightRequested * scale);
    }

    // Stop drive motors and return to RUN_USING_ENCODER
    private void stopAndReturnToEncoderMode() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Helper: convert inches to encoder ticks (wheel-rotation-based)
    private int inchesToTicks(double inches) {
        double mm = inches * MM_PER_INCH;
        double revs = mm / WHEEL_CIRCUMFERENCE_MM;
        return (int) Math.round(revs * TICKS_PER_REV);
    }
}
