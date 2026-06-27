package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This class handles the shooter macro and manual control using a custom software PIDF.
 * Includes linear interpolation for distance-based velocity.
 */
public class ShooterMechanism {
    private RobotHardware robot;
    
    // Constants for easy tuning
    public static final double GATE_CLOSED = 0.09;
    public static final double GATE_OPEN   = 0.2;
    public static final long   GATE_WAIT_MS = 300; 
    public static final long   INTAKE_MS   = 1300;

    // Velocity Calibration Threshold
    public static final double VEL_THRESHOLD = 0.95; // 95% of target

    // Linear Interpolation Table (Distance in inches, Velocity in ticks/sec)
    // Every 5 inches, velocity increases by 20 ticks/sec as placeholder
    private static final double[][] VELOCITY_TABLE = {
        {0,   400},
        {5,   420},
        {10,  440},
        {15,  460},
        {20,  480},
        {25,  500},
        {30,  520},
        {35,  540},
        {40,  560},
        {45,  580},
        {50,  600},
        {55,  620},
        {60,  640},
        {65,  660},
        {70,  680},
        {75,  700},
        {80,  720},
        {85,  740},
        {90,  760},
        {95,  780},
        {100, 800}
    };

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
    private double currentMacroTarget = 0;

    public ShooterMechanism(RobotHardware robot) {
        this.robot = robot;
        timer.reset();
    }

    /**
     * Calculates the target velocity based on distance using linear interpolation.
     * @param distance Distance to the goal in inches.
     * @return Target velocity in ticks/second.
     */
    public double getInterpolatedVelocity(double distance) {
        // Handle out-of-bounds distances
        if (distance <= VELOCITY_TABLE[0][0]) return VELOCITY_TABLE[0][1];
        if (distance >= VELOCITY_TABLE[VELOCITY_TABLE.length - 1][0]) return VELOCITY_TABLE[VELOCITY_TABLE.length - 1][1];

        // Find the interval
        for (int i = 0; i < VELOCITY_TABLE.length - 1; i++) {
            if (distance >= VELOCITY_TABLE[i][0] && distance <= VELOCITY_TABLE[i+1][0]) {
                double dist0 = VELOCITY_TABLE[i][0];
                double dist1 = VELOCITY_TABLE[i+1][0];
                double vel0  = VELOCITY_TABLE[i][1];
                double vel1  = VELOCITY_TABLE[i+1][1];

                // Linear Interpolation Formula: y = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
                return vel0 + (distance - dist0) * (vel1 - vel0) / (dist1 - dist0);
            }
        }
        return VELOCITY_TABLE[0][1]; // Fallback
    }

    public void startSequence(double distance) {
        if (currentState == State.IDLE) {
            currentMacroTarget = getInterpolatedVelocity(distance);
            currentState = State.MACRO_SPIN_UP;
            stateTimer.reset();
        }
    }

    /**
     * Internal PID calculation.
     */
    private double calculatePIDPower(double targetVel) {
        double rawVel = (robot.leftShootMotor.getVelocity() + robot.rightShootMotor.getVelocity()) / 2.0;
        double currentVel = (velFilter * lastVel) + ((1 - velFilter) * rawVel);
        lastVel = currentVel;

        double error = targetVel - currentVel;
        double dt = timer.seconds();
        timer.reset();

        double pPower = Range.clip(error * kP, -0.05, 1.0);
        double fPower = targetVel * kF;
        double rawDerivative = (dt > 0) ? (error - lastError) / dt : 0;
        double filteredDerivative = (dFilter * lastDerivative) + ((1 - dFilter) * rawDerivative);
        double dPower = filteredDerivative * kD;
        
        lastError = error;
        lastDerivative = filteredDerivative;

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
                power = calculatePIDPower(currentMacroTarget);
                robot.leftShootMotor.setPower(power);
                robot.rightShootMotor.setPower(power);
                robot.leftGateServo.setPosition(GATE_CLOSED);
                
                if (getFlywheelSpeed() >= currentMacroTarget * VEL_THRESHOLD) {
                    currentState = State.MACRO_GATE_WAIT;
                    stateTimer.reset();
                }
                break;

            case MACRO_GATE_WAIT:
                power = calculatePIDPower(currentMacroTarget);
                robot.leftShootMotor.setPower(power);
                robot.rightShootMotor.setPower(power);
                robot.leftGateServo.setPosition(GATE_OPEN);
                
                if (stateTimer.milliseconds() >= GATE_WAIT_MS) {
                    currentState = State.MACRO_SHOOTING;
                    stateTimer.reset();
                }
                break;

            case MACRO_SHOOTING:
                power = calculatePIDPower(currentMacroTarget);
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
