// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor;
  
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(RobotConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    // intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);

  }

  @Override
  public void periodic() {
    if (LoggingConstants.INTAKE_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      SmartDashboard.putNumber("intake_cmd", intakeMotor.getAppliedOutput());
      SmartDashboard.putNumber("intake_current", intakeMotor.getOutputCurrent());
      SmartDashboard.putNumber("intake_speed", intakeMotor.getEncoder().getVelocity());
    }

    // SmartDashboard.putBoolean("intake_beamBreak", hasBall());

    if (LoggingConstants.INTAKE_LEVEL.ordinal() >= LoggingLevel.COMPETITION.ordinal()) {}
  }
  
  public void set(double speed) {
    intakeMotor.set(speed);
  }

  public void in() {
    set(IntakeConstants.INTAKE_IN_SPEED);
  }

  public void inForShoot() {
    set(IntakeConstants.INTAKE_IN_FOR_SHOOTING_SPEED);
  }

  public void out() {
    set(IntakeConstants.INTAKE_OUT_SPEED);
  }
}
