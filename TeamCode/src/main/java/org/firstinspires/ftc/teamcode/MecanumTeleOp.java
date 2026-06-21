package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mecanum TeleOp", group="Linear OpMode")
public class MecanumTeleOp extends LinearOpMode {

    // Create a RobotHardware object
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        // Initialize the robot hardware
        robot.init();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // 1. DRIVE: POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double yaw     =  gamepad1.right_stick_x;
            
            robot.driveRobot(axial, lateral, yaw);

            // 2. INTAKE: Control using triggers with 0.05 deadzone
            double rt = (gamepad1.right_trigger > 0.05) ? gamepad1.right_trigger : 0;
            double lt = (gamepad1.left_trigger > 0.05) ? gamepad1.left_trigger : 0;
            double intakePower = rt - lt;
            
            robot.setIntakePower(intakePower);

            // 3. SHOOTER: Control using Dpad (Left = 30%, Right = 60%, Otherwise Off)
            double shooterPower = 0;
            if (gamepad1.dpad_left) {
                shooterPower = 0.3;
            } else if (gamepad1.dpad_right) {
                shooterPower = 0.6;
            }
            
            robot.setShooterPower(shooterPower);

            // 4. PINPOINT: Update and show acceleration
            robot.updatePinpoint();

            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Shooter Power", "%.2f", shooterPower);
            telemetry.addLine("--- Pinpoint Data (Inches) ---");
            telemetry.addData("X Position", "%.2f", robot.getPosX());
            telemetry.addData("Y Position", "%.2f", robot.getPosY());
            telemetry.addData("Heading", "%.2f°", robot.getHeading());
            telemetry.addData("Front/Back Accel", "%.2f in/s^2", robot.getXAcceleration());
            telemetry.addData("Lateral Accel", "%.2f in/s^2", robot.getYAcceleration());
            telemetry.update();
        }
    }
}
