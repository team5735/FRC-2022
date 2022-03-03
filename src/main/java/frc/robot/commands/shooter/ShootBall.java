// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterWheelsSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBall extends SequentialCommandGroup {
  /** Creates a new ShootBall. */
  public ShootBall(double speed, double angle, ShooterWheelsSubsystem shooterWheelsSubsystem, HoodSubsystem hoodSubsystem, FeederSubsystem feederSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> (shooterWheelsSubsystem.atSpeed() && hoodSubsystem.atSetpoint())),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
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
