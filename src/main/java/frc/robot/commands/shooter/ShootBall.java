// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import javax.sound.midi.Soundbank;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootBall extends CommandBase {
  private final ShooterSubsystem subsystem;

  public ShootBall(ShooterSubsystem subsystem) {
    this.subsystem = subsystem;

    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    subsystem.setBigShooter(0);
    subsystem.setSmallShooter(0);
  }

  @Override
  public void execute() {
    double leftTriggerValue = RobotContainer.subsystemController.getLeftTriggerAxis();
    double bigWheelSpeed = (leftTriggerValue < 0.05 && leftTriggerValue > -0.05) ? 0.6 : leftTriggerValue;
    subsystem.setBigShooter(bigWheelSpeed);
    double rightTriggerValue = RobotContainer.subsystemController.getRightTriggerAxis();
    double smallWheelSpeed = (rightTriggerValue < 0.05 && rightTriggerValue > -0.05) ? 0.6 : rightTriggerValue;
    subsystem.setBigShooter(smallWheelSpeed);
    subsystem.feederForward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
