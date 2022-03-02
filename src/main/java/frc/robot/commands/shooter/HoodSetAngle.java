// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class HoodSetAngle extends CommandBase {
  private final ShooterSubsystem subsystem;
  private final PIDController pidController;
  /** Creates a new HoodSetAngle. */
  public HoodSetAngle(ShooterSubsystem subsystem, double setpoint) {

    this.pidController = new PIDController(0.8, 0, 0.005);
    this.pidController.setSetpoint(setpoint);
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(subsystem.getHoodAngle());
    subsystem.setHoodSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //subsystem.setHoodSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}