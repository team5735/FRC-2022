// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.shooter.ShooterIdle;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class ShooterWheelsSubsystem extends SubsystemBase {
  private TalonFX bigShooterMasterMotor, bigShooterFollowerMotor, smallShooterMotor;
  
  public ShooterWheelsSubsystem() {
    bigShooterMasterMotor = new TalonFX(RobotConstants.BIG_SHOOTER_MASTER_MOTOR_ID);
    bigShooterMasterMotor.setInverted(true);
    bigShooterMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    bigShooterMasterMotor.config_kP(0, ShooterConstants.BIG_WHEEL_KP);
    bigShooterMasterMotor.config_kI(0, ShooterConstants.BIG_WHEEL_KI);
    bigShooterMasterMotor.config_kD(0, ShooterConstants.BIG_WHEEL_KD);
    bigShooterMasterMotor.config_kF(0, ShooterConstants.BIG_WHEEL_KF);
    bigShooterFollowerMotor = new TalonFX(RobotConstants.BIG_SHOOTER_FOLLOWER_MOTOR_ID);
    bigShooterFollowerMotor.follow(bigShooterMasterMotor);

    smallShooterMotor = new TalonFX(RobotConstants.SMALL_SHOOTER_MOTOR_ID);
    smallShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    smallShooterMotor.config_kP(0, ShooterConstants.SMALL_WHEEL_KP);
    smallShooterMotor.config_kI(0, ShooterConstants.SMALL_WHEEL_KI);
    smallShooterMotor.config_kD(0, ShooterConstants.SMALL_WHEEL_KD);
    smallShooterMotor.config_kF(0, ShooterConstants.BIG_WHEEL_KF);

    // setDefaultCommand(new ShooterIdle(this));
  }

  @Override
  public void periodic() {
    if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      SmartDashboard.putNumber("big_shooter_cmd", bigShooterMasterMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("small_shooter_cmd", smallShooterMotor.getMotorOutputPercent());
    }

    if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.COMPETITION.ordinal()) {
      SmartDashboard.putNumber("big_shooter_vel", bigShooterMasterMotor.getSelectedSensorVelocity());
      SmartDashboard.putNumber("small_shooter_vel", smallShooterMotor.getSelectedSensorVelocity());
    }
  }

  public void setBigShooter(double speed) {
    bigShooterMasterMotor.set(ControlMode.Velocity, speed);
  }

  public void setSmallShooter(double speed) {
    smallShooterMotor.set(ControlMode.Velocity, speed);
  }

  public void set(double speed) {
    setBigShooter(speed);
    setSmallShooter(speed * SmartDashboard.getNumber("vision_speed_ratio", ShooterConstants.SMALL_BIG_SPEED_RATIO));
  }

  public void reverse() {
    setPercent(-0.5);
  }
  
  public void setBigShooterPercent(double percent) {
    bigShooterMasterMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setSmallShooterPercent(double percent) {
    smallShooterMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setPercent(double percent) {
    setBigShooterPercent(percent);
    setSmallShooterPercent(percent);
  }

  // public void testPercent() {
  //   setPercent(0.5);
  // }

  public void stopShooter() {
    setBigShooter(0);
    setSmallShooter(0);
  }

  public boolean atSpeed() {
    return bigShooterMasterMotor.getClosedLoopError() < ShooterConstants.BIG_ERROR_TOLERANCE &&
      smallShooterMotor.getClosedLoopError() < ShooterConstants.SMALL_ERROR_TOLERANCE;
  }
}
