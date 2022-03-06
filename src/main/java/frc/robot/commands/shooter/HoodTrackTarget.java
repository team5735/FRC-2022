// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class HoodTrackTarget extends CommandBase {
  private HoodSubsystem subsystem;
  /** Creates a new HoodTrackTarget. */
  public HoodTrackTarget(HoodSubsystem subsystem) {
    addRequirements(subsystem);
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double angle = SmartDashboard.getNumber("hood_angle", 0.1);
    subsystem.setAngle(angle);
    subsystem.startHood();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = SmartDashboard.getNumber("hood_angle", 0.1);
    subsystem.setAngle(angle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
