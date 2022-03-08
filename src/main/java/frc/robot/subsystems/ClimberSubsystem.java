// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax winchMotor;
  private final double DEADBAND = 0.05;

  //I don't know if this controller setup works, but it should be okay for the moment
  private XboxController controller = new XboxController(0);


  public ClimberSubsystem() {
    winchMotor = new CANSparkMax(RobotConstants.WINCH_MOTOR_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    if (LoggingConstants.CLIMBER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      SmartDashboard.putNumber("winch_cmd", winchMotor.getAppliedOutput());
      SmartDashboard.putNumber("winch_current", winchMotor.getOutputCurrent());
    }
    double output = deadband(controller.getLeftY()); 
          winchMotor.set(output);
    // winchMotor.set(output);
    // System.out.println(output);
    // if (winchMotor.getEncoder().getPosition() >= -496.858) {
    //   winchMotor.set(-0.5);
    // }
    // else {
    //   winchMotor.set(0);
    // }
    SmartDashboard.putNumber("cmd", winchMotor.getAppliedOutput());
    SmartDashboard.putNumber("intake_current", winchMotor.getOutputCurrent());
    SmartDashboard.putNumber("intake_speed", winchMotor.getEncoder().getVelocity());
    // System.out.println(winchMotor.getEncoder().getCountsPerRevolution());
    System.out.println(winchMotor.getEncoder().getPosition());
    //Inital Position: -19.024
    //Final Position: -496.858
    //Distance:

    if (LoggingConstants.CLIMBER_LEVEL.ordinal() >= LoggingLevel.COMPETITION.ordinal()) {}
  }
  public double deadband(double value) {
    if(Math.abs(value) < DEADBAND) {
      return 0;
    } else {
      return value;
    }
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
