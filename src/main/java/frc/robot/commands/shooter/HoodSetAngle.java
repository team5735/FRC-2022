// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class HoodSetAngle extends CommandBase {

  private HoodSubsystem subsystem;
  private double angle;
  /** Creates a new HoodSetAngle. */
  public HoodSetAngle(HoodSubsystem subsystem, double angle) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
    this.angle = angle;
  }

  @Override
  public void initialize() {
    subsystem.setAngle(angle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
