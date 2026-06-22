package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This class handles the shooter macro and manual control logic using PID Velocity.
 */
public class ShooterMechanism {
    private RobotHardware robot;
    
    // Constants for easy tuning
    public static final double GATE_CLOSED = 0.09;
    public static final double GATE_OPEN   = 0.2;
    public static final long   SPIN_UP_MS  = 2000; 
    public static final long   GATE_WAIT_MS = 300; 
    public static final long   INTAKE_MS   = 1300;

    // Velocity Targets (Ticks per second)
    public static final double VEL_MACRO = 1500;
    public static final double VEL_LOW   = 800;
    public static final double VEL_HIGH  = 1800;

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
    }

    public void startSequence() {
        if (currentState == State.IDLE) {
            currentState = State.MACRO_SPIN_UP;
            stateTimer.reset();
        }
    }

    public double update(double targetVelocity) {
        double macroIntakePower = 0;

        switch (currentState) {
            case IDLE:
                robot.setShooterVelocity(0);
                robot.leftGateServo.setPosition(GATE_CLOSED);
                
                if (targetVelocity > 0) {
                    currentState = State.MANUAL_SPIN_UP;
                    stateTimer.reset();
                }
                break;

            case MANUAL_SPIN_UP:
                robot.setShooterVelocity(targetVelocity);
                robot.leftGateServo.setPosition(GATE_CLOSED);
                
                if (targetVelocity <= 0) {
                    currentState = State.IDLE;
                } else if (stateTimer.milliseconds() >= SPIN_UP_MS) {
                    currentState = State.MANUAL_READY;
                }
                break;

            case MANUAL_READY:
                robot.setShooterVelocity(targetVelocity);
                robot.leftGateServo.setPosition(GATE_OPEN);
                
                if (targetVelocity <= 0) {
                    currentState = State.IDLE;
                }
                break;

            case MACRO_SPIN_UP:
                robot.setShooterVelocity(VEL_MACRO);
                robot.leftGateServo.setPosition(GATE_CLOSED);
                
                if (stateTimer.milliseconds() >= SPIN_UP_MS) {
                    currentState = State.MACRO_GATE_WAIT;
                    stateTimer.reset();
                }
                break;

            case MACRO_GATE_WAIT:
                robot.setShooterVelocity(VEL_MACRO);
                robot.leftGateServo.setPosition(GATE_OPEN);
                
                if (stateTimer.milliseconds() >= GATE_WAIT_MS) {
                    currentState = State.MACRO_SHOOTING;
                    stateTimer.reset();
                }
                break;

            case MACRO_SHOOTING:
                robot.setShooterVelocity(VEL_MACRO);
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
