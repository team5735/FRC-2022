// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class LoggingConstants {
    public enum LoggingLevel {
        NONE,
        COMPETITION,
        DEBUG
    }

    public static final LoggingLevel INTAKE_LEVEL = LoggingLevel.COMPETITION;
    public static final LoggingLevel FEEDER_LEVEL = LoggingLevel.COMPETITION;
    public static final LoggingLevel SHOOTER_LEVEL = LoggingLevel.COMPETITION;
    public static final LoggingLevel CLIMBER_LEVEL = LoggingLevel.COMPETITION;
    public static final LoggingLevel DRIVING_LEVEL = LoggingLevel.COMPETITION;
}
