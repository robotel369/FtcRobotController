package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Shooter PIDF Tuner", group="Tuning")
public class PIDFTuningOpMode extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    double targetVelocity = 0;
    double p = 0.0; // Start at 0 to find F first
    double i = 0.0; // Keep at 0 to prevent runaway
    double d = 0.0;
    double f = 12.0;
    double increment = 1.0;

    double lastTargetVelocity = 0;
    ElapsedTime spinupTimer = new ElapsedTime();
    boolean isSpinningUp = false;
    double lastSpinupTime = 0;

    boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false, lastLB = false, lastRB = false;

    @Override
    public void runOpMode() {
        robot.init();
        
        // Hard reset motors
        robot.leftShootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightShootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        updatePIDF();

        telemetry.addLine("Ready to tune Shooter PIDF");
        telemetry.addLine("1. Start with P=0, F=0");
        telemetry.addLine("2. Increase F until actual speed matches target");
        telemetry.addLine("3. Add P to snap back after shooting");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Velocity Targets
            if (gamepad1.y) targetVelocity = 1000;
            if (gamepad1.x) targetVelocity = 1500;
            if (gamepad1.a) {
                targetVelocity = 0;
                // Force stop to break runaway
                robot.leftShootMotor.setPower(0);
                robot.rightShootMotor.setPower(0);
            }

            // Increment Tuning
            if (gamepad1.right_bumper && !lastRB) increment *= 10.0;
            if (gamepad1.left_bumper && !lastLB)  increment /= 10.0;
            lastRB = gamepad1.right_bumper;
            lastLB = gamepad1.left_bumper;

            // F/P Tuning
            boolean changed = false;
            if (gamepad1.dpad_up && !lastUp)      { f += increment; changed = true; }
            if (gamepad1.dpad_down && !lastDown)  { f -= increment; changed = true; }
            if (gamepad1.dpad_right && !lastRight) { p += increment; changed = true; }
            if (gamepad1.dpad_left && !lastLeft)   { p -= increment; changed = true; }
            
            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
            lastRight = gamepad1.dpad_right; lastLeft = gamepad1.dpad_left;

            // Spin-up Timer Logic
            if (targetVelocity != lastTargetVelocity) {
                if (targetVelocity > 0) {
                    spinupTimer.reset();
                    isSpinningUp = true;
                    lastSpinupTime = 0; // Clear result for new test
                } else {
                    isSpinningUp = false;
                    lastSpinupTime = 0; // Reset when target is 0
                }
                lastTargetVelocity = targetVelocity;
            }

            // Also restart timer if tuning changes while running
            if (changed && targetVelocity > 0) {
                spinupTimer.reset();
                isSpinningUp = true;
                lastSpinupTime = 0;
            }

            if (isSpinningUp) {
                double avgVel = (Math.abs(robot.leftShootMotor.getVelocity()) + Math.abs(robot.rightShootMotor.getVelocity())) / 2.0;
                if (avgVel >= targetVelocity * 0.98) {
                    lastSpinupTime = spinupTimer.milliseconds();
                    isSpinningUp = false;
                }
            }

            if (changed) updatePIDF();

            // Apply Velocity
            if (targetVelocity > 0) {
                robot.leftShootMotor.setVelocity(targetVelocity);
                robot.rightShootMotor.setVelocity(targetVelocity);
            } else {
                robot.leftShootMotor.setVelocity(0);
                robot.rightShootMotor.setVelocity(0);
            }

            // Read back data from both Hubs
            PIDFCoefficients hubL = robot.leftShootMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients hubR = robot.rightShootMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("TARGET VEL", targetVelocity);
            telemetry.addData("Spinup Time", isSpinningUp ? "Calculating..." : String.format("%.0f ms", lastSpinupTime));
            telemetry.addLine("--- Motor L ---");
            double leftVel = robot.leftShootMotor.getVelocity();
            telemetry.addData("Vel", "%.0f", leftVel);
            telemetry.addData("Pwr", "%.3f", robot.leftShootMotor.getPower());
            telemetry.addLine("--- Motor R ---");
            telemetry.addData("Vel", "%.0f", leftVel); // Duplicated from Left
            telemetry.addData("Pwr", "%.3f", robot.rightShootMotor.getPower());
            
            telemetry.addLine("--- PIDF (Live) ---");
            telemetry.addData("P", "%.2f (HubL: %.2f)", p, hubL.p);
            telemetry.addData("F", "%.2f (HubL: %.2f)", f, hubL.f);
            telemetry.addData("Step", increment);
            
            // ERROR CHECKING
            if (targetVelocity > 0) {
                if (robot.leftShootMotor.getVelocity() < 0) telemetry.addLine("!!! ERROR: L ENCODER REVERSED !!!");
                if (robot.rightShootMotor.getVelocity() < 0) telemetry.addLine("!!! ERROR: R ENCODER REVERSED !!!");
            }
            
            telemetry.update();
        }
    }

    private void updatePIDF() {
        robot.leftShootMotor.setVelocityPIDFCoefficients(p, i, d, f);
        robot.rightShootMotor.setVelocityPIDFCoefficients(p, i, d, f);
    }
}
