// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX bigShooterMasterMotor, bigShooterFollowerMotor, smallShooterMotor;
  private CANSparkMax feederMotor, hoodMotor;

  private DutyCycleEncoder hoodEncoder;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    bigShooterMasterMotor = new TalonFX(RobotConstants.BIG_SHOOTER_MASTER_MOTOR_ID);
    bigShooterMasterMotor.setInverted(true);
    bigShooterFollowerMotor = new TalonFX(RobotConstants.BIG_SHOOTER_FOLLOWER_MOTOR_ID);
    bigShooterFollowerMotor.follow(bigShooterMasterMotor);

    smallShooterMotor = new TalonFX(RobotConstants.SMALL_SHOOTER_MOTOR_ID);

    feederMotor = new CANSparkMax(RobotConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    feederMotor.setInverted(true);

    hoodMotor = new CANSparkMax(RobotConstants.HOOD_MOTOR_ID, MotorType.kBrushless);
    hoodMotor.setInverted(true);
    hoodMotor.setSmartCurrentLimit(RobotConstants.HOOD_MOTOR_CURRENT_LIMIT);  // Limit hood motor amp limit to prevent breaking hood
    hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 0.2f);
    hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, -0.03f);

    hoodEncoder = new DutyCycleEncoder(RobotConstants.HOOD_ENCODER_DIO_PORT);
  }

  @Override
  public void periodic() {
    if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      SmartDashboard.putNumber("big_shooter_cmd", bigShooterMasterMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("small_shooter_cmd", smallShooterMotor.getMotorOutputPercent());
    }

    if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.COMPETITION.ordinal()) {
      SmartDashboard.putNumber("hood_angle", getHoodAngle());
      SmartDashboard.putNumber("hood_raw_angle", hoodEncoder.getAbsolutePosition());
    }
  }

  public void setBigShooter(double speed) {
    bigShooterMasterMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setSmallShooter(double speed) {
    smallShooterMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setHoodSpeed(double speed) {
    hoodMotor.set(speed);
  }

  public void stopShooter() {
    setBigShooter(0);
    setSmallShooter(0);
    setHoodSpeed(0);
  }

  public void feederForward() {
    feederMotor.set(ShooterConstants.FEEDER_FORWARD_SPEED);
  }

  public void feederReverse() {
    feederMotor.set(ShooterConstants.FEEDER_REVERSE_SPEED);
  }

  public void feederStop() {
    feederMotor.set(0);
  }

  public void testHood() {
    setHoodSpeed(0.1);
  }

  public double getHoodAngle() {
    return 1-((hoodEncoder.getAbsolutePosition() + ShooterConstants.HOOD_ENCODER_OFFSET) % 1);
  }

}
