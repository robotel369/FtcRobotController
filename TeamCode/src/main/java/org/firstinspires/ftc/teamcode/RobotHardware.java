package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This class defines all the hardware for the robot and provides methods to control mechanisms.
 */
public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;

    // Drive Motors
    public DcMotor leftFront    = null;
    public DcMotor rightFront   = null;
    public DcMotor leftBack     = null;
    public DcMotor rightBack    = null;

    // Intake Motors
    public DcMotor leftIntake   = null;
    public DcMotor rightIntake  = null;

    // Shooter Motors
    public DcMotor leftShootMotor  = null;
    public DcMotor rightShootMotor = null;

    // Gate Servo
    public Servo leftGateServo = null;

    // Pinpoint Odometry
    public GoBildaPinpointDriver pinpoint = null;

    // Acceleration calculation
    private double lastXVel = 0;
    private double lastYVel = 0;
    private ElapsedTime accelTimer = new ElapsedTime();
    private double xAccel = 0;
    private double yAccel = 0;

    // Constants for easy tuning
    public static final double GATE_CLOSED = 0.09;
    public static final double GATE_OPEN   = 0.2;
    public static final long   SHOOT_DELAY_MS = 2000; // Time for motors to spin up

    private ElapsedTime shooterTimer = new ElapsedTime();
    private boolean shooterActive = false;

    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     */
    public void init() {
        // Drive Motors
        leftFront   = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        rightFront  = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = myOpMode.hardwareMap.get(DcMotor.class, "leftBack");
        rightBack   = myOpMode.hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Intake Motors
        leftIntake  = myOpMode.hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = myOpMode.hardwareMap.get(DcMotor.class, "rightIntake");

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        // Shooter Motors
        leftShootMotor  = myOpMode.hardwareMap.get(DcMotor.class, "leftShootMotor");
        rightShootMotor = myOpMode.hardwareMap.get(DcMotor.class, "rightShootMotor");

        leftShootMotor.setDirection(DcMotor.Direction.FORWARD);
        rightShootMotor.setDirection(DcMotor.Direction.REVERSE);

        // Gate Servo
        leftGateServo = myOpMode.hardwareMap.get(Servo.class, "leftGateServo");
        leftGateServo.setPosition(GATE_CLOSED);

        // Pinpoint initialization
        try {
            pinpoint = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(-3.3, -6.6, DistanceUnit.INCH); // Example offsets in inches
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            myOpMode.telemetry.addData("Pinpoint", "Not found or error: " + e.getMessage());
        }

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Drive the robot using mecanum kinematics.
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        double max;

        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
    }

    /**
     * Set the power for the intake motors.
     */
    public void setIntakePower(double power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    /**
     * Set the power for the shooter motors and handle the gate servo logic.
     * This should be called every loop in TeleOp.
     */
    public void setShooterPower(double power) {
        leftShootMotor.setPower(power);
        rightShootMotor.setPower(power);

        if (power > 0) {
            // If we just started shooting, reset the timer
            if (!shooterActive) {
                shooterTimer.reset();
                shooterActive = true;
            }
            
            // After the delay, open the gate
            if (shooterTimer.milliseconds() >= SHOOT_DELAY_MS) {
                leftGateServo.setPosition(GATE_OPEN);
            } else {
                leftGateServo.setPosition(GATE_CLOSED);
            }
        } else {
            // Power is 0, close the gate immediately
            leftGateServo.setPosition(GATE_CLOSED);
            shooterActive = false;
        }
    }

    /**
     * Updates the pinpoint sensor and calculates acceleration.
     * Call this every loop.
     */
    public void updatePinpoint() {
        if (pinpoint == null) return;
        
        pinpoint.update();
        
        double dt = accelTimer.seconds();
        if (dt > 0.001) { // Prevent division by very small dt
            double currentXVel = pinpoint.getVelX(DistanceUnit.INCH);
            double currentYVel = pinpoint.getVelY(DistanceUnit.INCH);
            
            xAccel = (currentXVel - lastXVel) / dt;
            yAccel = (currentYVel - lastYVel) / dt;
            
            lastXVel = currentXVel;
            lastYVel = currentYVel;
            accelTimer.reset();
        }
    }

    public double getXAcceleration() { return xAccel; }
    public double getYAcceleration() { return yAccel; }
    
    public double getPosX() { return (pinpoint != null) ? pinpoint.getPosX(DistanceUnit.INCH) : 0; }
    public double getPosY() { return (pinpoint != null) ? pinpoint.getPosY(DistanceUnit.INCH) : 0; }
    public double getHeading() { return (pinpoint != null) ? pinpoint.getHeading(AngleUnit.DEGREES) : 0; }
}
