package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="Melonbotics Analog Encoder Example", group="Examples")
public class MelonboticsEncoderExample extends LinearOpMode {

    private AnalogInput melonEncoder;

    // Constants for conversion
    private static final double MAX_VOLTAGE = 3.3;     // Melonbotics encoder max voltage
    private static final double FULL_ROTATION_DEG = 360.0;

    // For velocity calculation
    private double lastAngle = 0;
    private double lastTimestamp = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Make sure your config name matches!
        melonEncoder = hardwareMap.get(AnalogInput.class, "melonEncoder");

        waitForStart();

        lastTimestamp = getRuntime();
        lastAngle = getAngleDegrees();

        while (opModeIsActive()) {

            double angle = getAngleDegrees();
            double velocity = getAngularVelocity(angle);

            telemetry.addData("Voltage", melonEncoder.getVoltage());
            telemetry.addData("Angle (deg)", angle);
            telemetry.addData("Angular Vel (deg/s)", velocity);
            telemetry.update();
        }
    }

    /**
     * Converts raw analog voltage to angle in degrees.
     */
    private double getAngleDegrees() {
        double voltage = melonEncoder.getVoltage();
        return (voltage / MAX_VOLTAGE) * FULL_ROTATION_DEG;
    }

    /**
     * Computes angular velocity in degrees/second.
     * Automatically handles wrap-around from 360 → 0.
     */
    private double getAngularVelocity(double currentAngle) {
        double currentTime = getRuntime();
        double deltaTime = currentTime - lastTimestamp;

        double deltaAngle = currentAngle - lastAngle;


        double velocity = deltaAngle / deltaTime;

        lastAngle = currentAngle;
        lastTimestamp = currentTime;

        return velocity;
    }
}