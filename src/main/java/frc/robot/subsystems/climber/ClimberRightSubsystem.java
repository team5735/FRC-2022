// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climber.ClimberRightCommand;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class ClimberRightSubsystem extends SubsystemBase {
  private CANSparkMax winchMotor;
  private CANSparkMax rotateMotor;

  public ClimberRightSubsystem() {
    winchMotor = new CANSparkMax(ClimberConstants.RIGHT_WINCH_MOTOR_ID, MotorType.kBrushless);
    // winchMotor.restoreFactoryDefaults();
    winchMotor.setInverted(true);
    rotateMotor = new CANSparkMax(ClimberConstants.RIGHT_ROTATE_MOTOR_ID, MotorType.kBrushless);
    rotateMotor.restoreFactoryDefaults();
    CommandScheduler.getInstance().setDefaultCommand(this, new ClimberRightCommand(this));
    
    // leftStartPosition = leftWinchMotor.getEncoder().getPosition();
    // rightStartPosition = rightWinchMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    //Inital Position: -19.024
    //Final Position: -496.858
    //Difference: 477.834 Ticks
    // System.out.println("RIGHT WINCH ENCODER: " + winchMotor.getEncoder().getPosition());

    // SmartDashboard.putNumber("climber_cmd", winchMotor.getAppliedOutput());
    // SmartDashboard.putNumber("climber_current", winchMotor.getOutputCurrent());
    // SmartDashboard.putNumber("climber_speed", winchMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("RIGHT WINCH ENCODER", winchMotor.getEncoder().getPosition());

    if (LoggingConstants.CLIMBER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      // //System.out.println(leftWinchMotor.getEncoder().getCountsPerRevolution());
      // //System.out.println(leftWinchMotor.getEncoder().getPosition());
    }

    if (LoggingConstants.CLIMBER_LEVEL.ordinal() >= LoggingLevel.COMPETITION.ordinal()) {}
  }

  public void set(double speed) {
    winchMotor.set(speed);
  }

  public void rotate(double speed) {
    rotateMotor.set(speed);
  }

  public void up() {
    set(ClimberConstants.WINCH_UP_SPEED);
  }

  public void down() {
    set(ClimberConstants.WINCH_DOWN_SPEED);
  }

  public void right() {}

  public void left() {}

}
