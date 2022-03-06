// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class HoodSetAngle extends CommandBase {

  private HoodSubsystem subsystem;
  private Supplier<Double> angleGetter;
  /** Creates a new HoodSetAngle. */
  public HoodSetAngle(HoodSubsystem subsystem, Supplier<Double> angleGetter) {
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.angleGetter = angleGetter;
  }

  @Override
  public void initialize() {
    subsystem.setAngle(angleGetter.get());
    subsystem.startHood();
  }

  @Override
  public void execute() {
    subsystem.setAngle(angleGetter.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
