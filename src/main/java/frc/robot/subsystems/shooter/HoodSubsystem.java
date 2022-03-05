// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.ShooterConstants;

public class HoodSubsystem extends PIDSubsystem {

  private CANSparkMax hoodMotor;

  private DutyCycleEncoder hoodEncoder;

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() {
    super(new PIDController(0.8, 0, 0.005));

    hoodMotor = new CANSparkMax(RobotConstants.HOOD_MOTOR_ID, MotorType.kBrushless);
    hoodMotor.setInverted(true);
    hoodMotor.setSmartCurrentLimit(RobotConstants.HOOD_MOTOR_CURRENT_LIMIT);  // Limit hood motor amp limit to prevent breaking hood
    hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 0.2f);
    hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, -0.03f);

    hoodEncoder = new DutyCycleEncoder(RobotConstants.HOOD_ENCODER_DIO_PORT);

    getController().setTolerance(ShooterConstants.HOOD_TOLERANCE);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    hoodMotor.set(output);    // TODO Might need to change to voltage for different battery voltages
  }

  @Override
  public double getMeasurement() {
    return 1-((hoodEncoder.getAbsolutePosition() + ShooterConstants.HOOD_ENCODER_OFFSET) % 1);
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void stopHood() {
    disable();
    hoodMotor.set(0);
  }

  public void setAngle(double angle) {
    setSetpoint(Math.max(Math.min(1, angle), 0));   // 0 < Angle < 1; TODO calculate for actual min and max
    enable();
  }
}
