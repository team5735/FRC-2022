// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.FieldRelative;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.shooter.RunBigWheel;
import frc.robot.commands.shooter.RunSmallWheel;
import frc.robot.commands.vision.TurnToTarget;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static boolean fieldRelative = false;
  public final Drivetrain swerveDrivetrain = new Drivetrain();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public final Vision vision = new Vision();

  public static final XboxController driveController = new XboxController(RobotConstants.DRIVE_CONTROLLER_PORT);
  public static final XboxController subsystemController = new XboxController(RobotConstants.SUBSYSTEM_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
  }

  private void configureSubsystemControllerBindings() {
    // left trigger to run small shooter wheel
    new JoystickButton(subsystemController, XboxController.Axis.kLeftTrigger.value)
      .whileActiveContinuous(new RunSmallWheel(shooterSubsystem, subsystemController.getRightTriggerAxis()));
    // right trigger to run big shooter wheel
    new JoystickButton(subsystemController, XboxController.Axis.kLeftTrigger.value)
      .whileActiveContinuous(new RunBigWheel(shooterSubsystem, subsystemController.getLeftTriggerAxis()));

    // left bumper to intake in
    new JoystickButton(subsystemController, Button.kLeftBumper.value)
      .whenPressed(new IntakeIn(intakeSubsystem))
      .whenReleased(new IntakeStop(intakeSubsystem));
  
    // 'A' button to aim, bind to cmd TurnToTarget
    // new JoystickButton(subsystemController, Button.kA.value)
    //   .whenPressed(new TurnToTarget(vision, swerveDrivetrain))
    //   .whenReleased(command, interruptible)
  }

}
