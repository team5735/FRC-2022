// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterWheelsSubsystem;

public class ShooterIdle extends CommandBase {
  private ShooterWheelsSubsystem subsystem;

  public ShooterIdle(ShooterWheelsSubsystem subsystem) {
    addRequirements(subsystem);
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.set(ShooterConstants.IDLE_RPM);
  }

  @Override
  public void execute() {
    subsystem.set(ShooterConstants.IDLE_RPM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
