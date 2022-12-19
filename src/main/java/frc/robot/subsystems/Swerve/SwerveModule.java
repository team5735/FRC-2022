// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 2048;
  private static final double kDriveGearRatio = (48./16.)*(16./28.)*(60./15.);
  private static final double kTurningGearRatio = (32./15.)*(60./15.);

  // private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  // private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final TalonFX driveMotor;
  private final TalonFX turningMotor;

  private final DutyCycleEncoder absoluteEncoder;
  private final double absoluteEncoderOffset;
  private final int module;

  /**
   * Constructs a SwerveModule.
   * @param absoluteEncoderOffset in radians
   */
  public SwerveModule(TalonFX driveMotor, TalonFX turningMotor, SwervePIDConfig pidConfig, 
  DutyCycleEncoder absoluteEncoder, double absoluteEncoderOffset, int module) {

    this.driveMotor = driveMotor;
    this.turningMotor = turningMotor;
    this.absoluteEncoder = absoluteEncoder;
    this.absoluteEncoderOffset = absoluteEncoderOffset;
    this.module = module;
    this.absoluteEncoder.reset();
    
    // Set motor PID
    this.driveMotor.config_kP(0, pidConfig.getDriveMotorPID().getKp());
    this.driveMotor.config_kI(0, pidConfig.getDriveMotorPID().getKi());
    this.driveMotor.config_kD(0, pidConfig.getDriveMotorPID().getKd());
    this.driveMotor.config_kF(0, pidConfig.getDriveMotorPID().getKf());
    this.driveMotor.selectProfileSlot(0, 0);

    this.turningMotor.config_kP(0, pidConfig.getTurningMotorPID().getKp());
    this.turningMotor.config_kI(0, pidConfig.getTurningMotorPID().getKi());
    this.turningMotor.config_kD(0, pidConfig.getTurningMotorPID().getKd());
    this.turningMotor.config_kF(0, pidConfig.getTurningMotorPID().getKf());
    this.turningMotor.selectProfileSlot(0, 0);

    this.turningMotor.configMotionAcceleration(60000); // 25000
    this.turningMotor.configMotionCruiseVelocity(45000); // 12500
  }

  // Unit conversion methods
  public double driveEncoderPulseToDistance(double encoderPulse) {
    return (encoderPulse * (2 * kWheelRadius * Math.PI) / kEncoderResolution / kDriveGearRatio) * 10;
  }

  public double distanceToDriveEncoderPulse(double distance) {
    return distance * kEncoderResolution / (2 * kWheelRadius * Math.PI) * kDriveGearRatio/10;
  }

  public double turningEncoderPulseToRadians(double encoderPulse) {
    return encoderPulse / kEncoderResolution / kTurningGearRatio * (2 * Math.PI);
  }

  public double radiansToTurningEncoderPulse(double angle) {
    return angle * kEncoderResolution * kTurningGearRatio / (2 * Math.PI);
  }

  public double absoluteEncoderToRadians(double rawEncoder) {
    // return (rawEncoder + 1 - absoluteEncoderOffset / (2 * Math.PI)) % 1 * (2 * Math.PI);
    return (rawEncoder - absoluteEncoderOffset) % 1 * (2 * Math.PI);
  }
  public double radiansToDegree(double radians){
    return radians *360 / (2 * Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      driveEncoderPulseToDistance(driveMotor.getSelectedSensorVelocity()),
      new Rotation2d(absoluteEncoderToRadians(absoluteEncoder.get()))
    );
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
boolean flag = true;
  public void setDesiredState(SwerveModuleState desiredState) {

    Double currentAngle = absoluteEncoder.get();
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState targetState =
        SwerveModuleState.optimize(desiredState, new Rotation2d(absoluteEncoderToRadians(currentAngle)));
    // Set drive motor to spin at target velocity
    driveMotor.set(ControlMode.Velocity, distanceToDriveEncoderPulse(targetState.speedMetersPerSecond));

    // Set turning motor to target angle
    double angleDifference = getDiff(targetState.angle.getRadians(),  absoluteEncoderToRadians(currentAngle));

    if (( angleDifference >= Math.PI/2 || angleDifference <= -(Math.PI/2))){
      if (LoggingConstants.DRIVING_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
        SmartDashboard.putNumber(module + "desiredState",radiansToDegree( desiredState.angle.getRadians()));
        SmartDashboard.putNumber(module + "targetState", radiansToDegree(targetState.angle.getRadians()));
        SmartDashboard.putNumber(module + "currentAngle", radiansToDegree(absoluteEncoderToRadians(currentAngle)));
        SmartDashboard.putNumber(module + "angleDifference", radiansToDegree(angleDifference));
      }
      flag = false;
    }

    turningMotor.set(ControlMode.MotionMagic, turningMotor.getSelectedSensorPosition() + radiansToTurningEncoderPulse(angleDifference));
  }
  private double getDiff (double target, double current){
    double diff = target - current;
    if (diff > Math.PI){
      if (LoggingConstants.DRIVING_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
        SmartDashboard.putNumber(module + "+diff",diff);
      }
      return diff -(2*Math.PI);
    }
    if (diff < -Math.PI){
      
      if (LoggingConstants.DRIVING_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
        SmartDashboard.putNumber(module + "-diff",diff);
      }
      return diff + (2*Math.PI);
    }
    
    if (LoggingConstants.DRIVING_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      SmartDashboard.putNumber(module + "diff",diff);
    }
    return diff;
  }

  public void stop() {
    driveMotor.set(ControlMode.Velocity, 0);
    turningMotor.set(ControlMode.MotionMagic, 0);
  }
}

