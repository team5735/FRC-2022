// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterWheelsSubsystem;

public class ShooterWheelsStop extends CommandBase {
  private ShooterWheelsSubsystem subsystem;
  /** Creates a new StopShooterWheels. */
  public ShooterWheelsStop(ShooterWheelsSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
