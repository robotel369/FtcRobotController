package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Modern Flywheel PIDF Tuner (Software Implementation)
 * Based on 2024-2025 "Into the Deep" best practices.
 */
@TeleOp(name="Shooter PIDF Tuner (Final)", group="Tuning")
public class PIDFTuningOpMode extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    // Target Velocity
    double targetVelocity = 0;
    
    // Tuning Parameters (Software PIDF)
    double kP = 0.02; // Proportional
    double kD = 0.0000; // Derivative
    double kF = 0.00085; // Feedforward
    double increment = 0.0001;

    // Velocity Filtering (to prevent clicking)
    double currentVel = 0;
    double lastVel = 0;
    double velFilter = 0.7; // 0 = no filter, 1 = total filter (no movement)

    // PID State
    double lastError = 0;
    double lastDerivative = 0;
    double dFilter = 0.8; // Smoothing for the D-term
    
    ElapsedTime timer = new ElapsedTime();
    
    // Spin-up Monitoring
    double lastTargetVelocity = 0;
    ElapsedTime spinupTimer = new ElapsedTime();
    boolean isSpinningUp = false;
    double lastSpinupTime = 0;

    // Debouncing
    boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false, lastLB = false, lastRB = false;

    @Override
    public void runOpMode() {
        robot.init();
        
        // Use RUN_WITHOUT_ENCODER to bypass Hub PID for maximum software control
        robot.leftShootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightShootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Ready to tune Shooter PIDF");
        telemetry.addLine("Triangle: 400 | Square: 500 | Cross: 0");
        telemetry.addLine("Dpad Up/Down: Tune F");
        telemetry.addLine("Dpad Left/Right: Tune P");
        telemetry.addLine("LT + Dpad Up/Down: Tune D");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // --- 1. SET TARGETS ---
            if (gamepad1.y) targetVelocity = 400;
            if (gamepad1.x) targetVelocity = 500;
            if (gamepad1.a) targetVelocity = 0;

            // --- 2. STEP SIZE ---
            if (gamepad1.right_bumper && !lastRB) increment *= 10.0;
            if (gamepad1.left_bumper && !lastLB)  increment /= 10.0;
            lastRB = gamepad1.right_bumper;
            lastLB = gamepad1.left_bumper;

            // --- 3. TUNE VALUES ---
            boolean tuneD = gamepad1.left_trigger > 0.5;

            // F Tuning (Up/Down)
            if (!tuneD) {
                if (gamepad1.dpad_up && !lastUp)      kF += increment;
                if (gamepad1.dpad_down && !lastDown)  kF -= increment;
            } else {
                // D Tuning (LT + Up/Down)
                if (gamepad1.dpad_up && !lastUp)      kD += increment;
                if (gamepad1.dpad_down && !lastDown)  kD -= increment;
            }
            
            // P Tuning (Left/Right)
            if (gamepad1.dpad_right && !lastRight) kP += increment;
            if (gamepad1.dpad_left && !lastLeft)   kP -= increment;
            
            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
            lastRight = gamepad1.dpad_right; lastLeft = gamepad1.dpad_left;

            // --- 4. CALCULATE VELOCITY & ERROR ---
            double rawVel = (robot.leftShootMotor.getVelocity() + robot.rightShootMotor.getVelocity()) / 2.0;
            
            // Low-Pass Filter on Velocity (prevents jitter)
            currentVel = (velFilter * lastVel) + ((1 - velFilter) * rawVel);
            lastVel = currentVel;

            double error = targetVelocity - currentVel;
            double dt = timer.seconds();
            timer.reset();

            // --- 5. PIDF LOGIC ---
            // Proportional (Clipped to prevent excessive reverse acceleration)
            double pPower = Range.clip(error * kP, -0.05, 1.0);

            // Feedforward (Base power for the target)
            double fPower = targetVelocity * kF;

            // Derivative (Dampens overshoot and helps braking)
            double rawDerivative = (dt > 0) ? (error - lastError) / dt : 0;
            double filteredDerivative = (dFilter * lastDerivative) + ((1 - dFilter) * rawDerivative);
            double dPower = filteredDerivative * kD;
            
            lastError = error;
            lastDerivative = filteredDerivative;

            // Combine and Clip
            double totalPower = fPower + pPower + dPower;
            
            // Allow limited negative power for active braking/deceleration
            totalPower = Range.clip(totalPower, -0.05, 1.0);

            // Apply Power
            robot.leftShootMotor.setPower(totalPower);
            robot.rightShootMotor.setPower(totalPower);

            // --- 6. TIMER LOGIC ---
            if (targetVelocity != lastTargetVelocity) {
                if (targetVelocity > 0) {
                    spinupTimer.reset();
                    isSpinningUp = true;
                    lastSpinupTime = 0;
                } else {
                    isSpinningUp = false;
                }
                lastTargetVelocity = targetVelocity;
            }
            if (isSpinningUp && currentVel >= targetVelocity * 0.98) {
                lastSpinupTime = spinupTimer.milliseconds();
                isSpinningUp = false;
            }

            // --- 7. TELEMETRY ---
            telemetry.addData("TARGET", "%.0f", targetVelocity);
            telemetry.addData("ACTUAL", "%.0f (Raw: %.0f)", currentVel, rawVel);
            telemetry.addData("POWER ", "%.3f", totalPower);
            telemetry.addData("SPINUP", isSpinningUp ? "Calculating..." : String.format("%.0f ms", lastSpinupTime));
            telemetry.addLine("--- Tuning (Step: " + increment + ") ---");
            telemetry.addData("[F] Dpad U/D", "%.6f", kF);
            telemetry.addData("[P] Dpad L/R", "%.6f", kP);
            telemetry.addData("[D] LT + U/D", "%.6f", kD);
            telemetry.addLine("--- Breakdown ---");
            telemetry.addData("P-Pwr", "%.3f", pPower);
            telemetry.addData("D-Pwr", "%.3f", dPower);
            telemetry.addData("F-Pwr", "%.3f", fPower);
            telemetry.update();
        }
    }
}
