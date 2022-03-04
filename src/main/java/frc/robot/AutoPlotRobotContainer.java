// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.AutoDriveCommand;
import frc.robot.commands.auto.PlotPathCommand;
import frc.robot.commands.drivetrain.FieldRelative;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.shooter.FeederForward;
import frc.robot.commands.shooter.FeederStop;
import frc.robot.commands.shooter.HoodSetAngle;
import frc.robot.commands.shooter.ShootBall;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.plotter.DataPoint;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterWheelsSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class AutoPlotRobotContainer extends RobotContainer {
  public static boolean isPlotting = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public AutoPlotRobotContainer() {
    super();

    // Configure the button bindings
    configureDriveControllerBindings();
    configureSubsystemControllerBindings();
  }

  private void configureDriveControllerBindings() {
    // right bumper => intake in
    new JoystickButton(driveController, Button.kRightBumper.value)
        .whenPressed(new IntakeIn(intakeSubsystem))
        .whenReleased(new IntakeStop(intakeSubsystem));

    // left bumper => intake out
    new JoystickButton(driveController, Button.kLeftBumper.value)
        .whenPressed(new IntakeOut(intakeSubsystem))
        .whenReleased(new IntakeStop(intakeSubsystem));

    // A button => set field centric to true
    new JoystickButton(driveController, Button.kA.value)
        .whenPressed(new FieldRelative(swerveDrivetrain, true));

    // B button => set to robot centric
    new JoystickButton(driveController, Button.kB.value)
        .whenPressed(new FieldRelative(swerveDrivetrain, false));

    // Y button => toggle path plotting
    new JoystickButton(driveController, Button.kY.value)
        .whenPressed(new PlotPathCommand(swerveDrivetrain));
  }

  private void configureSubsystemControllerBindings() {

    // new JoystickButton(subsystemController, Button.kA.value)
    // .whenPressed(new InstantCommand(shooterSubsystem::testHood,
    // shooterSubsystem))
    // .whenReleased(new InstantCommand(shooterSubsystem::stopShooter,
    // shooterSubsystem));

    // new JoystickButton(subsystemController, Button.kB.value)
    // .whenPressed(new HoodSetAngle(HoodSubsystem, 0.75));

    // new JoystickButton(subsystemController, Button.kA.value)
    // .whenPressed(new HoodSetAngle(shooterSubsystem, 0.15));

    new JoystickButton(subsystemController, Button.kRightBumper.value)
      .whileActiveOnce(new ShootBall(5000, 0.45, shooterWheelsSubsystem, hoodSubsystem, feederSubsystem));

    new JoystickButton(subsystemController, Button.kA.value)
        .whenPressed(new InstantCommand(()->shooterWheelsSubsystem.set(15000), shooterWheelsSubsystem))
        .whenReleased(new InstantCommand(shooterWheelsSubsystem::stopShooter, shooterWheelsSubsystem));
    new JoystickButton(subsystemController, Button.kX.value)
        .whenPressed(new InstantCommand(()->shooterWheelsSubsystem.set(17000), shooterWheelsSubsystem))
        .whenReleased(new InstantCommand(shooterWheelsSubsystem::stopShooter, shooterWheelsSubsystem));

    new JoystickButton(subsystemController, Button.kY.value)
        .whenPressed(new FeederForward(feederSubsystem))
        .whenReleased(new FeederStop(feederSubsystem));

    new JoystickButton(subsystemController, Button.kB.value)
      .whenPressed(new HoodSetAngle(hoodSubsystem, 0.75));

    new JoystickButton(subsystemController, Axis.kLeftTrigger.value)
        .whenPressed(new InstantCommand(()->shooterWheelsSubsystem.set(subsystemController.getLeftTriggerAxis()), shooterWheelsSubsystem))
        .whenReleased(new InstantCommand(shooterWheelsSubsystem::stopShooter, shooterWheelsSubsystem));
    // // // left bumper to intake in
    // new JoystickButton(subsystemController, Button.kLeftBumper.value)
    // .whenPressed(new IntakeIn(intakeSubsystem))
    // .whenReleased(new IntakeStop(intakeSubsystem));

    // 'A' button to aim, bind to cmd TurnToTarget
    // new JoystickButton(subsystemController, Button.kA.value)
    // .whenPressed(new TurnToTarget(vision, swerveDrivetrain))
    // .whenReleased(command, interruptible)
  }

}
