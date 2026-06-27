package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Mecanum TeleOp", group="Linear OpMode")
public class MecanumTeleOp extends LinearOpMode {

    // Create a RobotHardware object
    RobotHardware robot = new RobotHardware(this);
    // Create a ShooterMechanism object
    ShooterMechanism shooter = new ShooterMechanism(robot);

    // Alliance / Goal Tracking
    private boolean isRedAlliance = false;
    private double goalX = 0;
    private double goalY = 0;
    private boolean lastL3 = false;

    @Override
    public void runOpMode() {
        // Initialize the robot hardware
        robot.init();
        
        // Set telemetry to HTML mode to allow colored text
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // 1. DRIVE: POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double yaw     =  gamepad1.right_stick_x;
            
            robot.driveRobot(axial, lateral, yaw);

            // 1.5 ALLIANCE TOGGLE: L3 switches between Blue (-72,0) and Red (72,0)
            if (gamepad1.left_stick_button && !lastL3) {
                isRedAlliance = !isRedAlliance;
                if (isRedAlliance) {
                    goalX = 72;
                    goalY = 0;
                } else {
                    goalX = -72;
                    goalY = 0;
                }
            }
            lastL3 = gamepad1.left_stick_button;

            // 2. SHOOTER: Manual (Dpad) or Macro (Circle/B)
            double shooterTargetVel = 0;
            if (gamepad1.dpad_left) {
                shooterTargetVel = 400;
            } else if (gamepad1.dpad_right) {
                shooterTargetVel = 500;
            }

            // Calculate distance to goal
            double dx = goalX - robot.getPosX();
            double dy = goalY - robot.getPosY();
            double distanceToGoal = Math.sqrt(dx*dx + dy*dy);

            if (gamepad1.b) {
                shooter.startSequence(distanceToGoal);
            }

            // Update shooter PID state machine
            double macroIntake = shooter.update(shooterTargetVel);

            // 3. INTAKE: Control using triggers or Macro
            double rt = (gamepad1.right_trigger > 0.05) ? gamepad1.right_trigger : 0;
            double lt = (gamepad1.left_trigger > 0.05) ? gamepad1.left_trigger : 0;
            
            // Use macro power if busy, otherwise use triggers
            double intakePower = shooter.isBusy() ? macroIntake : (rt - lt);
            robot.setIntakePower(intakePower);

            // 4. PINPOINT: Update and show acceleration
            robot.updatePinpoint();

            // Telemetry
            if (isRedAlliance) {
                telemetry.addLine("<font color='#FF0000'>Red Alliance</font>");
            } else {
                telemetry.addLine("<font color='#0000FF'>Blue Alliance</font>");
            }
            telemetry.addData("Interpolated Vel", "%.0f", shooter.getInterpolatedVelocity(distanceToGoal));
            telemetry.addData("Distance to Goal", "%.2f in", distanceToGoal);
            telemetry.addData("Status", "Running");
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Shooter Target Vel", "%.0f", shooterTargetVel);
            telemetry.addData("Flywheel Speed", "%.0f", shooter.getFlywheelSpeed());
            telemetry.addLine("--- Pinpoint Data (Inches) ---");
            telemetry.addData("X Position", "%.2f", robot.getPosX());
            telemetry.addData("Y Position", "%.2f", robot.getPosY());
            telemetry.addData("Heading", "%.2f°", robot.getHeading());
            telemetry.addLine("--- Robot Velocity (in/s) ---");
            telemetry.addData("X Velocity", "%.2f", robot.getVelX());
            telemetry.addData("Y Velocity", "%.2f", robot.getVelY());
            telemetry.update();
        }
    }
}
