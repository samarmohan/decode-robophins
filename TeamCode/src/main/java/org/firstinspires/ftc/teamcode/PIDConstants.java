package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class PIDConstants {
    public static final double LIMELIGHT_ROTATION_kP = 0.01, LIMELIGHT_ROTATION_kI = 0.0, LIMELIGHT_ROTATION_kD = 0, LIMELIGHT_ROTATION_kF = 0.1, LIMELIGHT_FF_DEADZONE = 0.75;
    public static final double ENCODER_ROTATION_kP = 0.0, ENCODER_ROTATION_kI = 0.0, ENCODER_ROTATION_kD = 0.0, ENCODER_ROTATION_kF = 0.000;
    public static final double FLYWHEEL_kP = 0.0015, FLYWHEEL_kI = 0.0, FLYWHEEL_kD = 0.0, FLYWHEEL_kF = 0.0002;
}