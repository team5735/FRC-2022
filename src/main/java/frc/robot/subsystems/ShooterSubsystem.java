// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX bigShooterMasterMotor, bigShooterFollowerMotor, smallShooterMotor;
  private CANSparkMax feederMotor, hoodMotor;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    bigShooterMasterMotor = new TalonFX(RobotConstants.BIG_SHOOTER_MASTER_MOTOR_ID);
    bigShooterMasterMotor.setInverted(true);
    bigShooterFollowerMotor = new TalonFX(RobotConstants.BIG_SHOOTER_FOLLOWER_MOTOR_ID);

    smallShooterMotor = new TalonFX(RobotConstants.SMALL_SHOOTER_MOTOR_ID);

    feederMotor = new CANSparkMax(RobotConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    feederMotor.setInverted(true);

    hoodMotor = new CANSparkMax(RobotConstants.HOOD_MOTOR_ID, MotorType.kBrushless);
    hoodMotor.setInverted(true);
    hoodMotor.setSmartCurrentLimit(RobotConstants.HOOD_MOTOR_CURRENT_LIMIT);  // Limit hood motor amp limit to prevent breaking hood
  }

  @Override
  public void periodic() {
    if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      SmartDashboard.putNumber("big_shooter_cmd", bigShooterMasterMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("small_shooter_cmd", smallShooterMotor.getMotorOutputPercent());

    }

    if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.COMPETITION.ordinal()) {}
  }

  public void setBigShooter(double speed) {
    bigShooterMasterMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setSmallShooter(double speed) {
    smallShooterMotor.set(ControlMode.PercentOutput, speed);
  }

  public void testBigShooter() {
    setBigShooter(0.05);
  }

  public void testSmallShooter() {
    setSmallShooter(0.1);
  }

  public void stopShooter() {
    setBigShooter(0);
    setSmallShooter(0);
  }
}
