// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterWheelsSubsystem;
import frc.robot.subsystems.shooter.SpeedAngle;
import frc.robot.subsystems.shooter.TargetMapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBall extends SequentialCommandGroup {
  ShooterWheelsSubsystem shooterWheelsSubsystem;
  HoodSubsystem hoodSubsystem;
  FeederSubsystem feederSubsystem;
  Vision visionSubSystem;
  /** Creates a new ShootBall. */
  public ShootBall(ShooterWheelsSubsystem shooterWheelsSubsystem, HoodSubsystem hoodSubsystem, FeederSubsystem feederSubsystem, Vision vision) {
    this.feederSubsystem =feederSubsystem;
    this.shooterWheelsSubsystem = shooterWheelsSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.visionSubSystem = vision;
    addCommands(
      new ParallelDeadlineGroup(
          new WaitUntilCommand(() -> (shooterWheelsSubsystem.atSpeed() && hoodSubsystem.atSetpoint())),
          new ParallelDeadlineGroup(
            new WaitCommand(3),
            new FeederForward(feederSubsystem)
          )
        ),
      new ParallelCommandGroup(
        new StopFeeder(feederSubsystem),
        new StopShooterWheels(shooterWheelsSubsystem)
      )
    );
  }

  @Override
  public void initialize() {
    super.initialize();
    double distance = visionSubSystem.getDistanceFromTargetInInches();
    SpeedAngle speedAngle = TargetMapper.getSpeedAngleByDistance(distance);
    if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      SmartDashboard.putNumber("Vision Distance", distance);
      SmartDashboard.putNumber("Target Speed", speedAngle.getSpeed());
      SmartDashboard.putNumber("Target Angle", speedAngle.getAngle());
      shooterWheelsSubsystem.set(speedAngle.getSpeed());
      hoodSubsystem.setSetpoint(speedAngle.getAngle());
    }
  }

}
