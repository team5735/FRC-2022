// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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

<<<<<<< HEAD
  public static final XboxController driveController = new XboxController(RobotConstants.DRIVE_CONTROLLER_PORT);
  public static final XboxController subsystemController = new XboxController(RobotConstants.SUBSYSTEM_CONTROLLER_PORT);
=======
  private final XboxController driveController = new XboxController(RobotConstants.DRIVE_CONTROLLER_PORT);
  private final XboxController subsystemController = new XboxController(RobotConstants.SUBSYSTEM_CONTROLLER_PORT);

>>>>>>> e1a06f6d9ffd4a71142e8d2942e0a0502c636953

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
<<<<<<< HEAD
    configureDriveButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriveButtonBindings() {
=======
    configureDriveControllerBindings();
    configureSubsystemControllerBindings();
  }

  private void configureDriveControllerBindings() {
>>>>>>> e1a06f6d9ffd4a71142e8d2942e0a0502c636953
    new JoystickButton(driveController, Button.kRightBumper.value)
      .whenPressed(new IntakeIn(intakeSubsystem))
      .whenReleased(new IntakeStop(intakeSubsystem));

    new JoystickButton(driveController, Button.kLeftBumper.value)
      .whenPressed(new IntakeOut(intakeSubsystem))
      .whenReleased(new IntakeStop(intakeSubsystem));
  }

<<<<<<< HEAD
=======
  private void configureSubsystemControllerBindings() {
    new JoystickButton(subsystemController, Button.kRightBumper.value)
      .whenPressed(new InstantCommand(shooterSubsystem::testShooter, shooterSubsystem))
      .whenReleased(new InstantCommand(shooterSubsystem::stopShooter, shooterSubsystem));
  }
>>>>>>> e1a06f6d9ffd4a71142e8d2942e0a0502c636953
}
