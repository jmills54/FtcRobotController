package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

/**
 * Simple mecanum teleop for GoBilda 312 motors (4x 312) with shooter, kicker and sorter control.
 * - Expects the following hardware in robot configuration:
 *     "frontLeft", "frontRight", "backLeft", "backRight",
 *     "shooterLeft", "shooterRight", "kickerRight", "kickerLeft", "sorter", "intake"
 * - Sets drivetrain motors to RUN_WITHOUT_ENCODER (typical for driver-controlled mecanum).
 * - Zero-power behavior is set to BRAKE for drivetrain. Shooter behavior left as default (can be changed).
 * - Motor directions: left side reversed (common wiring convention). Adjust if your robot spins.
 * - Drivetrain output is scaled so the maximum motor power is 0.75 (75%).
 * - Hold left trigger (gamepad1) for 25% precision drive mode (implemented).
 * - Press gamepad2 right bumper to toggle shooters on/off and to control kickerRight+sorter with a delayed start.
 * - Press gamepad2 left bumper to toggle shooters on/off and to control kickerLeft+sorter with a delayed start.
 *   When either kicker starts, sorter runs in the opposite direction of that kicker. For left-bumper-started kickerLeft,
 *   the sorter direction is reversed relative to the right-kicker case.
 *   If the corresponding bumper turns shooters OFF, the kicker and sorter are stopped immediately (pending starts canceled).
 * - Pressing gamepad2.b at any time immediately powers off shooterLeft/shooterRight/kickerLeft/kickerRight/sorter and cancels pending starts.
 * - Intake control is on controller 1 bumpers: gamepad1 left bumper runs the intake IN (positive direction);
 *   gamepad1 right bumper runs the intake OUT (opposite direction). If both bumpers are pressed the intake will remain stopped.
 * - Pressing gamepad2.x (rising edge) while the shooters are running will decrease the current shooter speed by 10%.
 * - Pressing gamepad2.a (rising edge) while the shooters are running will increase the current shooter speed by 10%.
 * - Pressing gamepad2.y (rising edge) will run a non-blocking stutter reverse routine on the shooters to try to dislodge a ball.
 *   The stutter temporarily overrides shooter outputs and restores their previous state when complete.
 */

@TeleOp (name = "2025_Decode_Competition")
public class DecodeCompetition2025 extends LinearOpMode {

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    // Shooter motors
    private DcMotor shooterLeft = null;
    private DcMotor shooterRight = null;

    // Intake motor
    private DcMotor intake = null;

    // Continuous servos
    private CRServo kickerRight = null;
    private CRServo kickerLeft = null;
    // single sorter servo used for both left/right kicker opposite-direction behavior
    private CRServo sorter = null;

    // Default maximum allowed motor output (0..1). Set to 0.75 per your request.
    private static final double DEFAULT_MAX_MOTOR_OUTPUT = 0.75;
    // Precision mode output when left trigger is held (0..1).
    private static final double PRECISION_MOTOR_OUTPUT = 0.25;
    // Deadzone threshold for trigger to engage precision mode
    private static final double TRIGGER_THRESHOLD = 0.05;

    // Base shooter power (initial)
    private static final double BASE_SHOOTER_POWER = 0.60;

    // Shooter current power (modifiable by gamepad2.x / gamepad2.a)
    private double shooterPowerCurrent = BASE_SHOOTER_POWER;
    // Increment / decrement step when gamepad2.a / gamepad2.x is pressed (10% of full scale)
    private static final double SHOOTER_STEP = 0.10;
    // Minimum allowed shooter power
    private static final double SHOOTER_MIN_POWER = 0.05;
    // Maximum allowed shooter power
    private static final double SHOOTER_MAX_POWER = 1.0;

    // Intake power maximum (0..1)
    private static final double INTAKE_POWER = 0.75;

    // Kicker/sorter power and delay (milliseconds)
    private static final double KICKER_POWER = 1.0; // full speed; change if you want slower
    private static final long KICKER_START_DELAY_MS = 2000; // 2 second delay

    // Stutter (reverse) routine parameters for gamepad2.y
    private static final int STUTTER_CYCLES = 4;             // number of reverse/pause cycles
    private static final long STUTTER_REVERSE_MS = 150;      // how long to reverse (ms) per cycle
    private static final long STUTTER_PAUSE_MS = 100;        // pause between reverses (ms)
    // reverse power fraction relative to current shooterPowerCurrent (negative to reverse)
    private static final double STUTTER_REVERSE_POWER_FRAC = 0.8;

    @Override
    public void runOpMode() {
        // Map drivetrain hardware
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Map shooter hardware
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");

        // Map intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Map kicker and sorter continuous servos (CRServo)
        kickerRight = hardwareMap.get(CRServo.class, "kickerRight");
        kickerLeft = hardwareMap.get(CRServo.class, "kickerLeft");
        sorter = hardwareMap.get(CRServo.class, "sorter");

        // Typical direction setup: left side reversed so positive power moves robot forward.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure shooter directions so they spin opposite when given the same positive power.
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure intake direction default. If your intake is reversed physically change this.
        // Positive power will be "IN" (bring game pieces into robot).
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // If you need kicker/sorter directions flipped, change here:
        kickerRight.setDirection(DcMotorSimple.Direction.REVERSE);
        kickerLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        sorter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Use RUN_WITHOUT_ENCODER for driver control.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Brake when power = 0 to help with precise stops for drivetrain
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Keep shooter float for flywheel behavior
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Ensure kicker and sorter and intake start stopped
        kickerRight.setPower(0.0);
        kickerLeft.setPower(0.0);
        sorter.setPower(0.0);
        intake.setPower(0.0);

        telemetry.addData("Status", "Ready");
        telemetry.addData("Note", "Default max motor output set to " + (int)(DEFAULT_MAX_MOTOR_OUTPUT * 100) + "%");
        telemetry.addData("Note2", "Hold left trigger (gamepad1) to enter 25% precision drive mode");
        telemetry.addData("Note3", "Press gamepad2 right bumper to toggle shooters at current shooterPower / control kickerRight+sorter with delayed start");
        telemetry.addData("Note4", "Press gamepad2 left bumper to toggle shooters at current shooterPower / control kickerLeft+sorter with delayed start (sorter direction reversed for left bumper)");
        telemetry.addData("Note5", "gamepad1 left bumper = intake IN, gamepad1 right bumper = intake OUT (opposite direction). Both pressed => intake stops.");
        telemetry.addData("Note6", "Press gamepad2.x to decrease shooter power by 10%%; press gamepad2.a to increase shooter power by 10%% while shooters are running.");
        telemetry.addData("Note7", "Press gamepad2.y to run a short stutter reverse to try to dislodge a ball (non-blocking).");
        telemetry.update();

        waitForStart();

        // Shooter toggle state and previous button states for edge detection
        boolean shooterOn = false;
        boolean prevRightBumper = false;
        boolean prevLeftBumper = false;
        // prev state for gamepad2.x, gamepad2.a, gamepad2.y to detect rising edge
        boolean prevX = false;
        boolean prevA = false;
        boolean prevY = false;

        // KickerRight/sorter state
        boolean kickerRightRunning = false;
        boolean kickerRightPendingStart = false;
        long kickerRightStartTimeMs = 0L; // when to start kickerRight+sorter

        // KickerLeft/sorter state (uses same sorter but runs sorter reversed when started by left bumper)
        boolean kickerLeftRunning = false;
        boolean kickerLeftPendingStart = false;
        long kickerLeftStartTimeMs = 0L; // when to start kickerLeft+sorter (sorter will run reversed)
        // Track which kicker last started the sorter to help avoid conflicts (right: 1, left: -1, 0: none)
        int sorterOwner = 0;

        // Stutter routine state (non-blocking)
        boolean stutterActive = false;
        long stutterPhaseStartMs = 0L;
        int stutterPhase = 0;            // 0..(STUTTER_CYCLES*2-1): even = reverse phase, odd = pause
        int stutterCyclesDone = 0;
        boolean shooterWasRunningBeforeStutter = false;

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            // Read joystick inputs
            double drive = -gamepad1.left_stick_y;    // forward/back
            double strafe = gamepad1.left_stick_x;    // left/right
            double rotate = gamepad1.right_stick_x;   // rotation
            double leftTrigger = gamepad1.left_trigger; // precision trigger (analog 0..1)

            // Mecanum wheel mixing
            double fl = drive + strafe + rotate;
            double fr = drive - strafe - rotate;
            double bl = drive - strafe + rotate;
            double br = drive + strafe - rotate;

            // Normalize wheel speeds so no magnitude exceeds 1.0
            double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            // Scale drivetrain outputs so max motor power is DEFAULT_MAX_MOTOR_OUTPUT (75%).
            // If precision mode is active (left trigger), further scale down to PRECISION_MOTOR_OUTPUT.
            double scale = DEFAULT_MAX_MOTOR_OUTPUT;
            if (leftTrigger > TRIGGER_THRESHOLD) {
                scale = PRECISION_MOTOR_OUTPUT;
            }

            double pFL = Range.clip(fl * scale, -DEFAULT_MAX_MOTOR_OUTPUT, DEFAULT_MAX_MOTOR_OUTPUT);
            double pFR = Range.clip(fr * scale, -DEFAULT_MAX_MOTOR_OUTPUT, DEFAULT_MAX_MOTOR_OUTPUT);
            double pBL = Range.clip(bl * scale, -DEFAULT_MAX_MOTOR_OUTPUT, DEFAULT_MAX_MOTOR_OUTPUT);
            double pBR = Range.clip(br * scale, -DEFAULT_MAX_MOTOR_OUTPUT, DEFAULT_MAX_MOTOR_OUTPUT);

            frontLeft.setPower(pFL);
            frontRight.setPower(pFR);
            backLeft.setPower(pBL);
            backRight.setPower(pBR);

            // --- Immediate emergency/off button (gamepad2.b) ---
            // If pressed at any time, immediately stop shooters and all kicker/sorter servos and cancel pending starts.
            if (gamepad2.b) {
                shooterOn = false;

                // Stop shooters
                shooterLeft.setPower(0.0);
                shooterRight.setPower(0.0);

                // Stop and cancel right kicker/sorter
                kickerRightRunning = false;
                kickerRightPendingStart = false;
                kickerRight.setPower(0.0);

                // Stop and cancel left kicker/sorter (uses same sorter)
                kickerLeftRunning = false;
                kickerLeftPendingStart = false;
                kickerLeft.setPower(0.0);

                // Stop sorter
                sorter.setPower(0.0);
                sorterOwner = 0;

                // Stop intake
                intake.setPower(0.0);

                // Cancel any stutter in progress
                stutterActive = false;

                // Telemetry immediate update
                telemetry.addData("EMERGENCY_STOP", "gamepad2.b pressed - all shooter/kicker/sorter/intake stopped");
                telemetry.update();

                // Continue to next loop iteration (drivers can still drive)
                continue;
            }

            // --- Gamepad2 right bumper logic (rising edge) ---
            boolean currentRightBumper = gamepad2.right_bumper;
            if (currentRightBumper && !prevRightBumper) {
                // Toggle shooter on/off
                shooterOn = !shooterOn;

                // If shooters were turned OFF by this toggle, also stop kickerRight and sorter immediately
                if (!shooterOn) {
                    kickerRightRunning = false;
                    kickerRightPendingStart = false;
                    kickerRight.setPower(0.0);
                    // If we were the owner of the sorter, stop it
                    if (sorterOwner == 1) {
                        sorter.setPower(0.0);
                        sorterOwner = 0;
                    }
                } else {
                    // If shooters turned ON, schedule delayed start for kickerRight+sorter (if not already running/pending)
                    if (!kickerRightRunning && !kickerRightPendingStart) {
                        kickerRightPendingStart = true;
                        kickerRightStartTimeMs = now + KICKER_START_DELAY_MS;
                    } else if (kickerRightRunning) {
                        kickerRightRunning = false;
                        kickerRightPendingStart = false;
                        kickerRight.setPower(0.0);
                        if (sorterOwner == 1) {
                            sorter.setPower(0.0);
                            sorterOwner = 0;
                        }
                    }
                }
            }
            prevRightBumper = currentRightBumper;

            // --- Gamepad2 left bumper logic (rising edge): kickerLeft+sorter control and shooter toggle ---
            boolean currentLeftBumper = gamepad2.left_bumper;
            if (currentLeftBumper && !prevLeftBumper) {
                // Toggle shooters on/off (shared shooterOn state)
                shooterOn = !shooterOn;

                // If shooters turned OFF by this left-bumper press, also stop kickerLeft and sorter immediately
                if (!shooterOn) {
                    kickerLeftRunning = false;
                    kickerLeftPendingStart = false;
                    kickerLeft.setPower(0.0);
                    // If sorter was owned by left kicker, stop it
                    if (sorterOwner == -1) {
                        sorter.setPower(0.0);
                        sorterOwner = 0;
                    }
                } else {
                    // If shooters turned ON, manage kickerLeft/sorterLeft:
                    // - If neither running nor pending start, schedule a delayed start for both.
                    // - If kickerLeft currently running, stop both immediately.
                    if (!kickerLeftRunning && !kickerLeftPendingStart) {
                        kickerLeftPendingStart = true;
                        kickerLeftStartTimeMs = now + KICKER_START_DELAY_MS;
                    } else if (kickerLeftRunning) {
                        kickerLeftRunning = false;
                        kickerLeftPendingStart = false;
                        kickerLeft.setPower(0.0);
                        if (sorterOwner == -1) {
                            sorter.setPower(0.0);
                            sorterOwner = 0;
                        }
                    }
                }
            }
            prevLeftBumper = currentLeftBumper;

            // --- Gamepad2.x: decrease shooter power by 10% on rising edge while shooters running ---
            boolean currentX = gamepad2.x;
            if (currentX && !prevX) {
                // Only adjust when shooters are running (shooterOn true)
                if (shooterOn) {
                    shooterPowerCurrent = Range.clip(shooterPowerCurrent - SHOOTER_STEP, SHOOTER_MIN_POWER, SHOOTER_MAX_POWER);
                    telemetry.addData("ShooterPowerAdjustedDown", "%.2f", shooterPowerCurrent);
                }
            }
            prevX = currentX;

            // --- Gamepad2.a: increase shooter power by 10% on rising edge while shooters running ---
            boolean currentA = gamepad2.a;
            if (currentA && !prevA) {
                // Only adjust when shooters are running (shooterOn true)
                if (shooterOn) {
                    shooterPowerCurrent = Range.clip(shooterPowerCurrent + SHOOTER_STEP, SHOOTER_MIN_POWER, SHOOTER_MAX_POWER);
                    telemetry.addData("ShooterPowerAdjustedUp", "%.2f", shooterPowerCurrent);
                }
            }
            prevA = currentA;

            // --- Gamepad2.y: start stutter reverse routine on rising edge (non-blocking) ---
            boolean currentY = gamepad2.y;
            if (currentY && !prevY) {
                // start stutter only if not already active
                if (!stutterActive) {
                    stutterActive = true;
                    stutterPhase = 0;
                    stutterPhaseStartMs = now;
                    stutterCyclesDone = 0;
                    shooterWasRunningBeforeStutter = shooterOn;
                    // we keep shooterOn state unchanged; stutter overrides outputs temporarily
                    telemetry.addData("Stutter", "Started");
                }
            }
            prevY = currentY;

            // Apply shooter power based on toggle and current shooterPowerCurrent,
            // except when stutterActive (stutter overrides shooter outputs).
            if (!stutterActive) {
                if (shooterOn) {
                    shooterLeft.setPower(shooterPowerCurrent);
                    shooterRight.setPower(shooterPowerCurrent);
                } else {
                    shooterLeft.setPower(0.0);
                    shooterRight.setPower(0.0);
                }
            } else {
                // Stutter state machine (non-blocking)
                // even phases: reverse; odd phases: pause
                long phaseElapsed = now - stutterPhaseStartMs;
                if ((stutterPhase % 2) == 0) {
                    // reverse phase
                    if (phaseElapsed >= STUTTER_REVERSE_MS) {
                        // end of reverse phase -> advance phase
                        stutterPhase++;
                        stutterPhaseStartMs = now;
                        // if we completed a reverse+pause (two phases), count a cycle when passing the pause end
                        if (stutterPhase % 2 == 1) {
                            // entering pause phase; do nothing special here
                        }
                    } else {
                        // within reverse: apply reverse power (fraction of current shooterPower)
                        double reversePower = -STUTTER_REVERSE_POWER_FRAC * shooterPowerCurrent;
                        shooterLeft.setPower(reversePower);
                        shooterRight.setPower(reversePower);
                    }
                } else {
                    // pause phase
                    if (phaseElapsed >= STUTTER_PAUSE_MS) {
                        // end of pause -> advance to next reverse or finish
                        stutterPhase++;
                        stutterPhaseStartMs = now;
                    } else {
                        // within pause: stop shooters
                        shooterLeft.setPower(0.0);
                        shooterRight.setPower(0.0);
                    }
                }

                // If we've advanced past the last phase, finish stutter
                if (stutterPhase >= STUTTER_CYCLES * 2) {
                    stutterActive = false;
                    // Restore shooters to their pre-stutter running state
                    if (shooterWasRunningBeforeStutter) {
                        shooterLeft.setPower(shooterPowerCurrent);
                        shooterRight.setPower(shooterPowerCurrent);
                    } else {
                        shooterLeft.setPower(0.0);
                        shooterRight.setPower(0.0);
                    }
                    telemetry.addData("Stutter", "Finished");
                }
            }

            // Handle kickerRight pending start / execution
            if (kickerRightPendingStart && now >= kickerRightStartTimeMs) {
                kickerRight.setPower(KICKER_POWER);
                // sorter runs opposite to the kickerRight
                sorter.setPower(-KICKER_POWER);
                kickerRightRunning = true;
                kickerRightPendingStart = false;
                sorterOwner = 1;
            }

            // Handle kickerLeft pending start / execution (start kickerLeft and sorter reversed vs right)
            if (kickerLeftPendingStart && now >= kickerLeftStartTimeMs) {
                kickerLeft.setPower(KICKER_POWER);
                // sorter runs reversed relative to the right-kicker case (per your request)
                sorter.setPower(KICKER_POWER);
                kickerLeftRunning = true;
                kickerLeftPendingStart = false;
                sorterOwner = -1;
            }

            // --- Intake control (controller 1 / gamepad1 bumpers) ---
            // Left bumper (gamepad1.left_bumper) => intake IN (positive direction) at full INTAKE_POWER
            // Right bumper (gamepad1.right_bumper) => intake OUT (opposite direction) at full INTAKE_POWER
            boolean intakeInBumper = gamepad1.left_bumper;
            boolean intakeOutBumper = gamepad1.right_bumper;

            // If both bumpers pressed, stop intake to avoid conflict
            if (intakeInBumper && !intakeOutBumper) {
                intake.setPower(INTAKE_POWER); // positive => IN
            } else if (intakeOutBumper && !intakeInBumper) {
                intake.setPower(-INTAKE_POWER); // negative => OUT
            } else {
                // neither bumper or both pressed -> stop intake
                intake.setPower(0.0);
            }

            // Telemetry for debugging
            telemetry.addData("LeftTrigger", "%.2f", leftTrigger);
            telemetry.addData("PrecisionModeActive", leftTrigger > TRIGGER_THRESHOLD ? "YES" : "NO");

            telemetry.addData("FL", "%.2f", frontLeft.getPower());
            telemetry.addData("FR", "%.2f", frontRight.getPower());
            telemetry.addData("BL", "%.2f", backLeft.getPower());
            telemetry.addData("BR", "%.2f", backRight.getPower());

            telemetry.addData("ShooterOn", shooterOn ? "YES" : "NO");
            telemetry.addData("ShooterPowerCurrent", "%.2f", shooterPowerCurrent);
            telemetry.addData("StutterActive", stutterActive ? "YES" : "NO");

            telemetry.addData("KickerRightRunning", kickerRightRunning ? "YES" : "NO");
            telemetry.addData("KickerRightPendingStart", kickerRightPendingStart ? "YES" : "NO");
            telemetry.addData("KickerRightStartTimeMs", kickerRightPendingStart ? "%d" : "n/a", kickerRightStartTimeMs);
            telemetry.addData("KickerRightPower", "%.2f", kickerRightRunning ? KICKER_POWER : 0.0);
            telemetry.addData("KickerLeftRunning", kickerLeftRunning ? "YES" : "NO");
            telemetry.addData("KickerLeftPendingStart", kickerLeftPendingStart ? "YES" : "NO");
            telemetry.addData("KickerLeftStartTimeMs", kickerLeftPendingStart ? "%d" : "n/a", kickerLeftStartTimeMs);
            telemetry.addData("KickerLeftPower", "%.2f", kickerLeftRunning ? KICKER_POWER : 0.0);

            telemetry.addData("SorterOwner", sorterOwner == 1 ? "RIGHT" : (sorterOwner == -1 ? "LEFT" : "NONE"));
            telemetry.addData("SorterPower", "%.2f", sorter.getPower());

            telemetry.addData("IntakeInBumper", intakeInBumper ? "YES" : "NO");
            telemetry.addData("IntakeOutBumper", intakeOutBumper ? "YES" : "NO");
            telemetry.addData("IntakePower", "%.2f", intake.getPower());

            telemetry.update();
        }

        // Ensure shooters, kicker and sorter and intake are stopped when opmode ends
        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
        kickerRight.setPower(0.0);
        kickerLeft.setPower(0.0);
        sorter.setPower(0.0);
        intake.setPower(0.0);
    }
}