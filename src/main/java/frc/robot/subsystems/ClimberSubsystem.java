// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax winchMotor;

  public ClimberSubsystem() {
    winchMotor = new CANSparkMax(RobotConstants.WINCH_MOTOR_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    if (LoggingConstants.CLIMBER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      SmartDashboard.putNumber("winch_cmd", winchMotor.getAppliedOutput());
      SmartDashboard.putNumber("winch_current", winchMotor.getOutputCurrent());
    }

    if (LoggingConstants.CLIMBER_LEVEL.ordinal() >= LoggingLevel.COMPETITION.ordinal()) {}
  }
  
  public void set(double speed) {
    winchMotor.set(speed);
  }

  public void up() {
    set(ClimberConstants.WINCH_UP_SPEED);
  }

  public void down() {
    set(ClimberConstants.WINCH_DOWN_SPEED);
  }
}
