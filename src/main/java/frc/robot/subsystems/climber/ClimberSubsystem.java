// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.climber.ClimberJoystickCommand;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class ClimberSubsystem extends SubsystemBase { //TODO: change to PID subsystem
  private CANSparkMax leftWinchMotor;
  private CANSparkMax rightWinchMotor;
  private CANSparkMax leftRotateMotor;
  private CANSparkMax rightRotateMotor;

  private final double DEADBAND = 0.05;
  private double leftStartPosition;
  private double rightStartPosition;

  public ClimberSubsystem() {
    leftWinchMotor = new CANSparkMax(ClimberConstants.LEFT_WINCH_MOTOR_ID, MotorType.kBrushless);
    rightWinchMotor = new CANSparkMax(ClimberConstants.RIGHT_WINCH_MOTRO_ID, MotorType.kBrushless);
    leftRotateMotor = new CANSparkMax(ClimberConstants.LEFT_ROTATE_MOTOR_ID, MotorType.kBrushless);
    rightRotateMotor = new CANSparkMax(ClimberConstants.RIGHT_ROTATE_MOTOR_ID, MotorType.kBrushless);
    
    CommandScheduler.getInstance().setDefaultCommand(this, new ClimberJoystickCommand(this));
    
    // leftStartPosition = leftWinchMotor.getEncoder().getPosition();
    // rightStartPosition = rightWinchMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    //Inital Position: -19.024
    //Final Position: -496.858
    //Difference: 477.834 Ticks

    SmartDashboard.putNumber("climber_cmd", leftWinchMotor.getAppliedOutput());
    SmartDashboard.putNumber("climber_current", leftWinchMotor.getOutputCurrent());
    SmartDashboard.putNumber("climber_speed", leftWinchMotor.getEncoder().getVelocity());

    if (LoggingConstants.CLIMBER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      // //System.out.println(leftWinchMotor.getEncoder().getCountsPerRevolution());
      // //System.out.println(leftWinchMotor.getEncoder().getPosition());
    }

    if (LoggingConstants.CLIMBER_LEVEL.ordinal() >= LoggingLevel.COMPETITION.ordinal()) {}
  }

  public void set(double speed) {
    leftWinchMotor.set(speed);
    rightWinchMotor.set(speed);
  }

  public void rotate(double speed) {
    leftRotateMotor.set(speed);
    rightRotateMotor.set(speed);
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
