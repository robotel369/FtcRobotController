package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class practice extends OpMode {
    public void init () {
    int teamNumber = 19900;
    double turretAngle = 0;
            telemetry.addData("turretAngle", turretAngle);
                    telemetry.addData("teamNumber", teamNumber);


    }

    @Override
    public void loop() {

    }
}


