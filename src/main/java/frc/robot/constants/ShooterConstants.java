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
    public static final double FEEDER_REVERSE_FOR_INTAKE_SPEED = -0.07;

    public static final double BIG_WHEEL_KP = 0.2;
    public static final double BIG_WHEEL_KI = 0;
    public static final double BIG_WHEEL_KD = 0;
    public static final double BIG_WHEEL_KF = 0.05;
    public static final double SMALL_WHEEL_KP = 0.1;
    public static final double SMALL_WHEEL_KI = 0;
    public static final double SMALL_WHEEL_KD = 0;
    public static final double SMALL_WHEEL_KF = 0.01;

    public static final double SMALL_BIG_SPEED_RATIO = 1.5; // Drives small wheel at speed * ratio

    public static final double HOOD_TOLERANCE = 0.03;

    public static final double BIG_ERROR_TOLERANCE = 100, SMALL_ERROR_TOLERANCE = 100;

    public static final long MIN_TURN_TIME = 500; //millis
    public static final long TURN_TIMEOUT = 2000;

}
