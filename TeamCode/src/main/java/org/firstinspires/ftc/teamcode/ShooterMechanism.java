package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This class handles the shooter macro and manual control using a custom software PIDF.
 */
public class ShooterMechanism {
    private RobotHardware robot;
    
    // Constants for easy tuning
    public static final double GATE_CLOSED = 0.09;
    public static final double GATE_OPEN   = 0.2;
    public static final long   SPIN_UP_MS  = 1;
    public static final long   GATE_WAIT_MS = 300; 
    public static final long   INTAKE_MS   = 1300;

    // Velocity Targets (Ticks per second)
    public static final double VEL_MACRO = 500;
    public static final double VEL_THRESHOLD = 0.95; // 95% of target

    // PIDF Coefficients (Tuned Values)
    private static final double kP = 0.02;
    private static final double kD = 0.0000;
    private static final double kF = 0.00085;
    
    // Filters
    private static final double velFilter = 0.7;
    private static final double dFilter = 0.8;

    // PID State
    private double lastError = 0;
    private double lastDerivative = 0;
    private double lastVel = 0;
    private ElapsedTime timer = new ElapsedTime();

    private enum State {
        IDLE,
        MANUAL_SPIN_UP,
        MANUAL_READY,
        MACRO_SPIN_UP,
        MACRO_GATE_WAIT,
        MACRO_SHOOTING
    }

    private State currentState = State.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();

    public ShooterMechanism(RobotHardware robot) {
        this.robot = robot;
        timer.reset();
    }

    public void startSequence() {
        if (currentState == State.IDLE) {
            currentState = State.MACRO_SPIN_UP;
            stateTimer.reset();
        }
    }

    /**
     * Internal PID calculation.
     * @param targetVel The target velocity in ticks/second.
     * @return Calculated power [-0.05, 1.0]
     */
    private double calculatePIDPower(double targetVel) {
        double rawVel = (robot.leftShootMotor.getVelocity() + robot.rightShootMotor.getVelocity()) / 2.0;
        
        // Low-Pass Filter on Velocity
        double currentVel = (velFilter * lastVel) + ((1 - velFilter) * rawVel);
        lastVel = currentVel;

        double error = targetVel - currentVel;
        double dt = timer.seconds();
        timer.reset();

        // Proportional (Clipped to prevent excessive reverse acceleration)
        double pPower = Range.clip(error * kP, -0.05, 1.0);

        // Feedforward
        double fPower = targetVel * kF;

        // Derivative (Dampens overshoot)
        double rawDerivative = (dt > 0) ? (error - lastError) / dt : 0;
        double filteredDerivative = (dFilter * lastDerivative) + ((1 - dFilter) * rawDerivative);
        double dPower = filteredDerivative * kD;
        
        lastError = error;
        lastDerivative = filteredDerivative;

        // Combine and Clip
        return Range.clip(fPower + pPower + dPower, -0.05, 1.0);
    }

    public double update(double targetVelocity) {
        double macroIntakePower = 0;
        double power;

        switch (currentState) {
            case IDLE:
                power = calculatePIDPower(0);
                robot.leftShootMotor.setPower(power);
                robot.rightShootMotor.setPower(power);
                robot.leftGateServo.setPosition(GATE_CLOSED);
                
                if (targetVelocity > 0) {
                    currentState = State.MANUAL_SPIN_UP;
                    stateTimer.reset();
                }
                break;

            case MANUAL_SPIN_UP:
                power = calculatePIDPower(targetVelocity);
                robot.leftShootMotor.setPower(power);
                robot.rightShootMotor.setPower(power);
                robot.leftGateServo.setPosition(GATE_CLOSED);
                
                if (targetVelocity <= 0) {
                    currentState = State.IDLE;
                } else if (getFlywheelSpeed() >= targetVelocity * VEL_THRESHOLD) {
                    currentState = State.MANUAL_READY;
                }
                break;

            case MANUAL_READY:
                power = calculatePIDPower(targetVelocity);
                robot.leftShootMotor.setPower(power);
                robot.rightShootMotor.setPower(power);
                robot.leftGateServo.setPosition(GATE_OPEN);
                
                if (targetVelocity <= 0) {
                    currentState = State.IDLE;
                }
                break;

            case MACRO_SPIN_UP:
                power = calculatePIDPower(VEL_MACRO);
                robot.leftShootMotor.setPower(power);
                robot.rightShootMotor.setPower(power);
                robot.leftGateServo.setPosition(GATE_CLOSED);
                
                if (getFlywheelSpeed() >= VEL_MACRO * VEL_THRESHOLD) {
                    currentState = State.MACRO_GATE_WAIT;
                    stateTimer.reset();
                }
                break;

            case MACRO_GATE_WAIT:
                power = calculatePIDPower(VEL_MACRO);
                robot.leftShootMotor.setPower(power);
                robot.rightShootMotor.setPower(power);
                robot.leftGateServo.setPosition(GATE_OPEN);
                
                if (stateTimer.milliseconds() >= GATE_WAIT_MS) {
                    currentState = State.MACRO_SHOOTING;
                    stateTimer.reset();
                }
                break;

            case MACRO_SHOOTING:
                power = calculatePIDPower(VEL_MACRO);
                robot.leftShootMotor.setPower(power);
                robot.rightShootMotor.setPower(power);
                robot.leftGateServo.setPosition(GATE_OPEN);
                macroIntakePower = 1.0;
                
                if (stateTimer.milliseconds() >= INTAKE_MS) {
                    currentState = State.IDLE;
                }
                break;
        }
        
        return macroIntakePower;
    }

    public boolean isBusy() {
        return currentState != State.IDLE && 
               currentState != State.MANUAL_SPIN_UP && 
               currentState != State.MANUAL_READY;
    }
    
    public double getFlywheelSpeed() {
        return (Math.abs(robot.leftShootMotor.getVelocity()) + Math.abs(robot.rightShootMotor.getVelocity())) / 2.0;
    }
}
