// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.FeederSubsystem;

public class FeederReverseForShoot extends CommandBase {
  private final FeederSubsystem subsystem;
  private int times = 0;

  public FeederReverseForShoot(FeederSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    subsystem.feederREverseForShoot();
    times = 0;
  }

  @Override
  public void execute() {
    times++;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return times == 5;
  }
}
