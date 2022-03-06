// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class HoodStop extends CommandBase {
  private HoodSubsystem subsystem;
  /** Creates a new HoodStop. */
  public HoodStop(HoodSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.stopHood();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
