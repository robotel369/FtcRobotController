package org.firstinspires.ftc.teamcode;

}
// Get analog port instance from hardwareMap
AnalogInput encoder = hardwareMap.get(AnalogInput.class, "encoder");

// Simple position return
double position = encoder.getVoltage() / 3.2 * 360;