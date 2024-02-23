package org.firstinspires.ftc.teamcode.Drivetrain;

public interface WheelRPMConfig {
    double [] RPMs = {215,
            230,
            212.8,
            230};
    double min = RPMs[2];
    double[] RPMMultipliers = { min / RPMs[0], min / RPMs[1], min / RPMs[2], min / RPMs[3]};
}
