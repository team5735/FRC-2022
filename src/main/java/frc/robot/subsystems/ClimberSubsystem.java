// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax leftWinchMotor;
  private CANSparkMax rightWinchMotor;

  private final double DEADBAND = 0.05;
  private final double startPosition = leftWinchMotor.getEncoder().getPosition();
  private int mode;

  public ClimberSubsystem() {
    leftWinchMotor = new CANSparkMax(ClimberConstants.LEFT_WINCH_MOTOR_ID, MotorType.kBrushless);
    rightWinchMotor = new CANSparkMax(ClimberConstants.RIGHT_WINCH_MOTRO_ID, MotorType.kBrushless);

    mode = 0;
  }

  @Override
  public void periodic() {
    // Controller Mode
    // mode 0 is manual control
    if (mode == 0) {
        double output = deadband(RobotContainer.subsystemController.getLeftY()); 
        leftWinchMotor.set(output);
        // System.out.println(output);
    }
    // 1/2 is a test value;
    // mode 1 is auto up
    if (mode == 1) {
        if (leftWinchMotor.getEncoder().getPosition() <= startPosition + 477.834) {
          leftWinchMotor.set(0.5);
        }
    }
    // mode 2 is auto down
    if (mode == 2) {
        if (leftWinchMotor.getEncoder().getPosition() >= startPosition) {
          leftWinchMotor.set(-0.5);
        }
    }
    //Inital Position: -19.024
    //Final Position: -496.858
    //Difference: 477.834 Ticks

    SmartDashboard.putNumber("climber_cmd", leftWinchMotor.getAppliedOutput());
    SmartDashboard.putNumber("climber_current", leftWinchMotor.getOutputCurrent());
    SmartDashboard.putNumber("climber_speed", leftWinchMotor.getEncoder().getVelocity());

    if (LoggingConstants.CLIMBER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      // System.out.println(leftWinchMotor.getEncoder().getCountsPerRevolution());
      // System.out.println(leftWinchMotor.getEncoder().getPosition());
    }

    if (LoggingConstants.CLIMBER_LEVEL.ordinal() >= LoggingLevel.COMPETITION.ordinal()) {}
  }

  public double deadband(double value) {
    if (Math.abs(value) < DEADBAND) {
      return 0;
    } else {
      return value;
    }
  }

  public void changeMode() {
    mode ++;
    if (mode > 2) {
      mode = 0;
    }
  }
  
  public void set(double speed) {
    leftWinchMotor.set(speed);
  }

  public void up() {
    set(ClimberConstants.WINCH_UP_SPEED);
  }

  public void down() {
    set(ClimberConstants.WINCH_DOWN_SPEED);
  }
}
