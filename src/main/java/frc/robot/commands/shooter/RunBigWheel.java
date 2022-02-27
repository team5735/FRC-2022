// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RunBigWheel extends CommandBase {
  private final ShooterSubsystem subsystem;
  private double wheelSpeed;

  public RunBigWheel(ShooterSubsystem subsystem, double speed) {
    this.subsystem = subsystem;
    this.wheelSpeed = speed;

    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    subsystem.setBigShooter(0);
  }

  @Override
  public void execute() {
    subsystem.setBigShooter(wheelSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
