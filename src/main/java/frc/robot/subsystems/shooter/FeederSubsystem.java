// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class FeederSubsystem extends SubsystemBase {
  private CANSparkMax feederMotor;

  public FeederSubsystem() {
    feederMotor = new CANSparkMax(RobotConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    feederMotor.setInverted(true);
    // feederMotor.setSmartCurrentLimit(limit)
  }

  @Override
  public void periodic() {
    if (LoggingConstants.FEEDER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      SmartDashboard.putNumber("feeder_cmd", feederMotor.getAppliedOutput());
      SmartDashboard.putNumber("feeder_current", feederMotor.getOutputCurrent());
    }

    if (LoggingConstants.FEEDER_LEVEL.ordinal() >= LoggingLevel.COMPETITION.ordinal()) {
      SmartDashboard.putNumber("feeder_dir", feederMotor.getAppliedOutput() == 0 ? (0) : (feederMotor.getAppliedOutput() < 0 ? -1 : 1));
    }
  }

  public void feederForward() {
    feederMotor.set(ShooterConstants.FEEDER_FORWARD_SPEED);
  }

  public void feederForwardForShoot() {
    feederMotor.set(ShooterConstants.FEEDER_FORWARD_FOR_SHOOTTING_SPEED);
  }

  public void feederReverse() {
    feederMotor.set(ShooterConstants.FEEDER_REVERSE_SPEED);
  }

  public void feederREverseForShoot() {
    feederMotor.set(ShooterConstants.FEEDER_REVERSE_FOR_SHOOTTING_SPEED);
  }

  public void feederPlusIntake() {
    feederMotor.set(ShooterConstants.FEEDER_REVERSE_FOR_INTAKE_SPEED);
  }

  public void feederStop() {
    feederMotor.set(0);
  }
}
