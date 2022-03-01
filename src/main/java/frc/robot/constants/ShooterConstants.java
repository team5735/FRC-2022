// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class ShooterConstants {
    // Set this value to the raw hoodEncoder.getAbsolutePosition() when the hood is at the top and bottom.
    private static final double HOOD_RAW_TOP = 0.659;
    private static final double HOOD_RAW_BOTTOM = 0.570;

    public static final double HOOD_ENCODER_OFFSET = 1-((HOOD_RAW_BOTTOM + HOOD_RAW_TOP) / 2);

    public static final double FEEDER_FORWARD_SPEED = 0.4;
    public static final double FEEDER_REVERSE_SPEED = -0.4;
}
