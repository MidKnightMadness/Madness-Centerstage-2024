package org.firstinspires.ftc.teamcode.Drivetrain;

public interface WheelRPMConfig {
    double [] RPMs = {209.2,
            180.3,
            206.1,
            184.6};
    double min = RPMs[3];
    double[] RPMMultipliers = { min / RPMs[0], min / RPMs[1] * 0.8, min / RPMs[2], min / RPMs[3] * 0.8};
}
