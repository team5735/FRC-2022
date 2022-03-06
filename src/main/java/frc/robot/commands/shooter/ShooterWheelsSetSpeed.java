// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterWheelsSubsystem;

public class ShooterWheelsSetSpeed extends CommandBase {
  private ShooterWheelsSubsystem subsystem;
  private double speed;

  public ShooterWheelsSetSpeed(ShooterWheelsSubsystem subsystem, double speed) {
    this.subsystem = subsystem;
    this.speed = speed;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.set(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
