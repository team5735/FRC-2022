// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.subsystems.shooter.TargetMapper;
import frc.robot.subsystems.shooter.SpeedAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBall extends SequentialCommandGroup {
  private Vision vision = new Vision();
  /** Creates a new ShootBall. */
  public ShootBall(double speed, double angle, ShooterWheelsSubsystem shooterWheelsSubsystem, HoodSubsystem hoodSubsystem, FeederSubsystem feederSubsystem) {
    double distance = vision.getDistanceFromTargetInInches();
    SpeedAngle speedAngle = TargetMapper.getSpeedAngleByDistance(distance);
    if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
      SmartDashboard.putNumber("Vision Distance", distance);
      SmartDashboard.putNumber("Target Speed", speedAngle.getSpeed());
      SmartDashboard.putNumber("Target Angle", speedAngle.getAngle());
    }
    addCommands(
      new ParallelCommandGroup(
        new InstantCommand(() -> shooterWheelsSubsystem.set(speedAngle.getSpeed()), shooterWheelsSubsystem),
        new InstantCommand(() -> hoodSubsystem.setSetpoint(speedAngle.getAngle()), hoodSubsystem)
      ),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> (shooterWheelsSubsystem.atSpeed() && hoodSubsystem.atSetpoint())),
          new ParallelDeadlineGroup(
            new WaitCommand(5),
            new FeederForward(feederSubsystem)
          )
        ),
        new RunShooterWheels(shooterWheelsSubsystem, speed),
        new HoodSetAngle(hoodSubsystem, angle)
      ),
      new ParallelCommandGroup(
        new StopFeeder(feederSubsystem),
        new StopShooterWheels(shooterWheelsSubsystem)
      )
    );
  }
}
