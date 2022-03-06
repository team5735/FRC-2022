// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterWheelsSubsystem;

public class ShooterWheelsSetSpeed extends CommandBase {
  private ShooterWheelsSubsystem subsystem;
  private Supplier<Double> speedGetter;

  public ShooterWheelsSetSpeed(ShooterWheelsSubsystem subsystem, Supplier<Double> speedGetter) {
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.speedGetter = speedGetter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.set(speedGetter.get());
  }

  @Override
  public void execute() {
    subsystem.set(speedGetter.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
