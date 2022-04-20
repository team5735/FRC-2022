// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.auto.AutoDriveCommand;
import frc.robot.commands.auto.AutoDrivePointCommand;
import frc.robot.commands.drivetrain.StopDrivetrainCommand;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeInForShoot;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.shooter.FeederForward;
import frc.robot.commands.shooter.FeederForwardForShoot;
import frc.robot.commands.shooter.FeederPlusIntakeIn;
import frc.robot.commands.shooter.FeederReverse;
import frc.robot.commands.shooter.FeederReverseForShoot;
import frc.robot.commands.shooter.FeederStop;
import frc.robot.commands.shooter.HoodSetAngle;
import frc.robot.commands.shooter.HoodStop;
import frc.robot.commands.shooter.ShooterWheelsAtSpeed;
import frc.robot.commands.shooter.ShooterWheelsReverse;
import frc.robot.commands.shooter.ShooterWheelsSetSpeed;
import frc.robot.commands.shooter.ShooterWheelsStop;
import frc.robot.commands.vision.TurnToTarget;

import frc.robot.commands.vision.TurnOffLimelightCommand;
import frc.robot.commands.vision.TurnOnLimelight;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.climber.ClimberLeftSubsystem;
import frc.robot.subsystems.climber.ClimberRightSubsystem;
import frc.robot.subsystems.plotter.AutoPath;
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
public class RobotContainer {
  public static boolean fieldRelative = false;
  public final Drivetrain swerveDrivetrain = new Drivetrain();

  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final ShooterWheelsSubsystem shooterWheelsSubsystem = new ShooterWheelsSubsystem();
  public final HoodSubsystem hoodSubsystem = new HoodSubsystem();
  public final FeederSubsystem feederSubsystem = new FeederSubsystem();
  public final ClimberLeftSubsystem climberLeftSubsystem = new ClimberLeftSubsystem();
  public final ClimberRightSubsystem climberRightSubsystem = new ClimberRightSubsystem();

  public final Vision vision = new Vision();

  public static final XboxController driveController = new XboxController(RobotConstants.DRIVE_CONTROLLER_PORT);
  public static final XboxController subsystemController = new XboxController(RobotConstants.SUBSYSTEM_CONTROLLER_PORT);

  public boolean isHoodUp = false;

  // Autonomous path generation related
  public static SendableChooser<String> autoPathChooser = new SendableChooser<>();
  public ArrayList<DataPoint> testAuto;
  public ArrayList<DataPoint> runItBack;
  public ArrayList<DataPoint> twoBallInitial;
  public ArrayList<DataPoint> turning;
  public ArrayList<DataPoint> slowTest;
  public ArrayList<DataPoint> twoBallAuto;

  public static boolean isPlotting = false;

  private Long startTime;
  private String fileTime;
  private DateTimeFormatter filenameFormatter;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    setupPathChooser();

    SmartDashboard.putNumber("manual_hood_angle", 0.6);
    SmartDashboard.putNumber("manual_shooter_speed", 11500);
    // SmartDashboard.putNumber("kP", 0);
    // SmartDashboard.putNumber("kI", 0);
    // SmartDashboard.putNumber("kD", 0);
    // SmartDashboard.putNumber("kF", 0.0475);
    // SmartDashboard.putNumber("kAccel", 50000);
    // SmartDashboard.putNumber("kVel", 25000);

    SmartDashboard.putString("DistanceTwoBall", "108");
    SmartDashboard.putString("DistanceManual", "75");

    // Configure the button bindings
    configureDriveControllerBindings();
    configureSubsystemControllerBindings();
  }

  public void readPaths() {
    runItBack = AutoPath.readAutoFile("runItBackNew.txt");
    twoBallInitial = AutoPath.readAutoFile("2BallDrive.txt");
    slowTest = AutoPath.readAutoFile("5ftTest.txt");
    twoBallAuto = AutoPath.readAutoFile("newTwoBallAuto.txt");
  }

  public void setupPathChooser() {
    String[] autoNames = {"Turns and Shoots", "Run It Back", "Two Ball", "Slow Test", "Two Ball Auto"};

    for (String pathName : autoNames) {
      autoPathChooser.addOption(pathName, pathName);
    }

    SmartDashboard.putData("Auto Path", autoPathChooser);
  }

  public Command getAutonomousCommand() {

    String autoPath = autoPathChooser.getSelected();
    Command turnToTarget = new TurnToTarget(vision, swerveDrivetrain);
    if (autoPath.equals("Run It Back")) {
      return new SequentialCommandGroup(
        // new HoodSetAngle(subsystem, angleGetter)
        new ParallelDeadlineGroup(new WaitCommand(0.5), new HoodSetAngle(hoodSubsystem, () -> (0.6))),
        new ParallelDeadlineGroup(
            new WaitCommand(2),
            new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
            new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0))), 
            new SequentialCommandGroup(new WaitCommand(1.6), new FeederForward(feederSubsystem))),
        new ParallelDeadlineGroup(new AutoDriveCommand(runItBack, swerveDrivetrain, fieldRelative), new IntakeIn(intakeSubsystem), 
        new FeederStop(feederSubsystem),
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem)), 
        new TurnToTarget(vision, swerveDrivetrain), 
        new ParallelDeadlineGroup(new WaitCommand(0.5), new StopDrivetrainCommand(swerveDrivetrain)),
        new SequentialCommandGroup(new ParallelDeadlineGroup(new WaitCommand(0.25), new ParallelCommandGroup(
          new FeederReverse(feederSubsystem),
          new ShooterWheelsReverse(shooterWheelsSubsystem)
        ))), 
          new ParallelCommandGroup(
            new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
            new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0))), 
            new SequentialCommandGroup(new WaitCommand(2), new FeederForward(feederSubsystem))),
        new WaitCommand(1),
        new FeederStop(feederSubsystem),
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem)
      );
    }

    else if(autoPath.equals("Two Ball Auto")) {
      return new SequentialCommandGroup(
        new ParallelDeadlineGroup(new WaitCommand(0.5), new HoodSetAngle(hoodSubsystem, () -> (0.6))),
        new ParallelDeadlineGroup(
          new AutoDriveCommand(twoBallAuto, swerveDrivetrain, fieldRelative),
          new ParallelCommandGroup(
            new IntakeIn(intakeSubsystem), 
            new FeederPlusIntakeIn(feederSubsystem)
          )
        ),
        new ParallelCommandGroup( 
          new IntakeStop(intakeSubsystem), 
          new FeederStop(feederSubsystem)
        ),
        new TurnToTarget(vision, swerveDrivetrain), 
        new ParallelDeadlineGroup(
          new WaitCommand(0.25),
          new StopDrivetrainCommand(swerveDrivetrain)
        ),
        new ParallelDeadlineGroup(
          new WaitCommand(0.25),
          new ParallelCommandGroup(
            new FeederReverse(feederSubsystem),
            new ShooterWheelsReverse(shooterWheelsSubsystem)
          )
        ),
        new FeederStop(feederSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem),
        new SequentialCommandGroup(
          new FeederReverseForShoot(feederSubsystem),
          new ParallelDeadlineGroup(
            new ShooterWheelsAtSpeed(shooterWheelsSubsystem),
            new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
            new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0)))
          ),
          new SequentialCommandGroup(
            new WaitCommand(1.5),
            new ParallelCommandGroup(
              new FeederForwardForShoot(feederSubsystem),
              new IntakeInForShoot(intakeSubsystem)
            )
          )
          
        )
      );
    }

    else if(autoPath.equals("Two Ball")) {
      return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new ParallelCommandGroup(
          new IntakeIn(intakeSubsystem) 
          //new FeederPlusIntakeIn(feederSubsystem)
          ),
          new HoodSetAngle(hoodSubsystem, () -> (0.6))
        ),
        new ParallelDeadlineGroup(
          new AutoDriveCommand(twoBallInitial, swerveDrivetrain, fieldRelative),
          new ParallelCommandGroup(
          new IntakeIn(intakeSubsystem)
          // FeederPlusIntakeIn(feederSubsystem)
          )
        ),
        new TurnToTarget(vision, swerveDrivetrain),
        new ParallelDeadlineGroup(
          new WaitCommand(0.75),
          new StopDrivetrainCommand(swerveDrivetrain)
        ),
        new WaitCommand(1),
        new IntakeStop(intakeSubsystem),
        new FeederStop(feederSubsystem),
        new SequentialCommandGroup(
          new WaitCommand(5),
          new FeederReverseForShoot(feederSubsystem),
          // new ParallelDeadlineGroup(
          //   new TurnToTarget(vision, swerveDrivetrain),
          //   new HoodSetAngle(hoodSubsystem, () -> (0.45)),
          //   new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> ((double)10000))),
          new ParallelDeadlineGroup(
            new ShooterWheelsAtSpeed(shooterWheelsSubsystem),
            new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
            new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0)))
          ),
          new SequentialCommandGroup(
            new WaitCommand(1.5),
            new ParallelCommandGroup(
              new FeederForwardForShoot(feederSubsystem),
              new IntakeInForShoot(intakeSubsystem)
            )
          )
          
        ),
        new WaitCommand(0.5),
        new IntakeStop(intakeSubsystem),
        new FeederStop(feederSubsystem),
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem)
      );
    }

    else if (autoPath.equals("Turns and Shoots")) {
      return new SequentialCommandGroup(
        new TurnToTarget(vision, swerveDrivetrain), 
        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new StopDrivetrainCommand(swerveDrivetrain)
        ),
      
       new ParallelCommandGroup(
          new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
          new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0))), 
          new SequentialCommandGroup(new WaitCommand(0.75), new FeederForward(feederSubsystem))),

        new WaitCommand(0.5),
        new FeederStop(feederSubsystem),
        new WaitCommand(0.25),
        new FeederForward(feederSubsystem)
      );
    }
    else if (autoPath.equals("Slow Test")) {
      return new AutoDrivePointCommand(slowTest, swerveDrivetrain, fieldRelative);
    }
    else {
      return new SequentialCommandGroup(new Command[] {});
    }
  }

  private void configureDriveControllerBindings() {
    // 'X' button to aim, bind to cmd TurnToTarget
    Command turnToTarget = new TurnToTarget(vision, swerveDrivetrain);
    new JoystickButton(driveController, Button.kX.value)
        .whenPressed(turnToTarget)
        .whenReleased(new CancelCommand(turnToTarget));

    // right bumper => intake in
    new JoystickButton(driveController, Button.kRightBumper.value)
        .whenPressed(new ParallelCommandGroup(
          new IntakeIn(intakeSubsystem), 
          new FeederPlusIntakeIn(feederSubsystem)))
        .whenReleased(new ParallelCommandGroup( 
          new IntakeStop(intakeSubsystem), 
          new FeederStop(feederSubsystem)));

    // left bumper => intake out
    new JoystickButton(driveController, Button.kLeftBumper.value)
        .whenPressed(new IntakeOut(intakeSubsystem))
        .whenReleased(new IntakeStop(intakeSubsystem));

    // 'A' button => One button shoot including ramp up and feeder (plus intake)
    new JoystickButton(driveController, Button.kA.value)
        .whenPressed(
          new SequentialCommandGroup(
          // new ParallelDeadlineGroup(
          //   new WaitCommand(0.5),
          //   new FeederReverseForShoot(feederSubsystem),
          //   new ShooterWheelsReverse(shooterWheelsSubsystem)
          //   ),
          // new ParallelDeadlineGroup(
          //   new TurnToTarget(vision, swerveDrivetrain),
          //   new HoodSetAngle(hoodSubsystem, () -> (0.45)),
          //   new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> ((double)10000))),
          
          new FeederReverseForShoot(feederSubsystem),
          new ParallelDeadlineGroup(
            new ShooterWheelsAtSpeed(shooterWheelsSubsystem),
            new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
            new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0)))
          ),
          new SequentialCommandGroup(
            new WaitCommand(1),
            new ParallelCommandGroup(
              new FeederForwardForShoot(feederSubsystem),
              new IntakeInForShoot(intakeSubsystem)
            )
          )
          
        ))
        .whenReleased(
          new ParallelCommandGroup(
            new FeederStop(feederSubsystem),
            new HoodStop(hoodSubsystem),
            new ShooterWheelsStop(shooterWheelsSubsystem),
            new IntakeStop(intakeSubsystem)
          )
        );

    // 'B' button +> stop drivetrain
    new JoystickButton(driveController, Button.kB.value).whenPressed(
        new StopDrivetrainCommand(swerveDrivetrain)
    );
  }

  private void configureSubsystemControllerBindings() {
    // right bumper => shoot base on vision values
    new JoystickButton(subsystemController, Button.kRightBumper.value)
      .whenPressed(new ParallelCommandGroup(
        new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
        new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0)))
      ))
      .whenReleased(new ParallelCommandGroup(
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem)
      ));

    // left bumper => shoot based on maunal values
    new JoystickButton(subsystemController, Button.kLeftBumper.value)
      .whenPressed(new ParallelCommandGroup(
        new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("manual_hood_angle", 0.6
        ))),
        new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("manual_shooter_speed", 11500)))
      ))
      .whenReleased(new ParallelCommandGroup(
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem)
      ));

    // A button => feeder and shooter reverse
    new JoystickButton(subsystemController, Button.kA.value)
      .whenPressed(new ParallelCommandGroup(
        new FeederReverse(feederSubsystem),
        new ShooterWheelsReverse(shooterWheelsSubsystem)
      ))
      .whenReleased(new ParallelCommandGroup(
        new FeederStop(feederSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem)
      ));

    // X button => feeder forward
    new JoystickButton(subsystemController, Button.kX.value)
        .whenPressed(
          new ParallelCommandGroup(
            new FeederForward(feederSubsystem)
          )
        )
        .whenReleased(
          new ParallelCommandGroup(
            new FeederStop(feederSubsystem)
          )
        );

    // left trigger => manual shooting based on trigger value
    new JoystickButton(subsystemController, Axis.kLeftTrigger.value)
        .whenPressed(new InstantCommand(
            ()->shooterWheelsSubsystem.set(subsystemController.getLeftTriggerAxis()), shooterWheelsSubsystem))
        .whenReleased(new InstantCommand(shooterWheelsSubsystem::stopShooter, shooterWheelsSubsystem));

    new JoystickButton(subsystemController, Button.kY.value)
        .whenPressed(new TurnOffLimelightCommand(vision))
        .whenReleased(new TurnOnLimelight(vision));
  }

  public void configureAutoButton() {
    if (driveController.getStartButtonPressed()){
      isPlotting = true;
      fileCreator();
    }

    if (driveController.getBackButtonPressed()) {
      isPlotting = false;
      fileCreator();
    }


  }

  private void fileCreator() { 
    filenameFormatter = DateTimeFormatter.ofPattern("dd-MM-yyyy-HH-mm-ss");

    //Starts and Stops plotting
    if (isPlotting) {
        try {
          fileTime = filenameFormatter.format(LocalDateTime.now());
          File myObj = new File("/U/" + fileTime + ".txt");
          startTime = System.currentTimeMillis();
          if (myObj.createNewFile()) {
            //System.out.println("File created: " + myObj.getName());
          } else {
            //System.out.println("File already exists.");
          }
        
        } catch (IOException e) {
          //System.out.println("An error occurred.");
          e.printStackTrace();
        }
        //SmartDashboard.putBoolean("Is Plotting", true);
      }
      else{
        //System.out.println("Stops plotting");
        //SmartDashboard.putBoolean("Is Plotting", false);
      }
  }

  public void drivingPlotter() {
      if (isPlotting) {
        Long time = System.currentTimeMillis() - startTime;
        double robotX = swerveDrivetrain.poseEstimator().getEstimatedPosition().getX();
        double robotY = swerveDrivetrain.poseEstimator().getEstimatedPosition().getY();
        double rotation = swerveDrivetrain.poseEstimator().getEstimatedPosition().getRotation().getDegrees();
        double actualRotation = swerveDrivetrain.getRot();
        double xSpeed = swerveDrivetrain.getXSpeed();
        double ySpeed = swerveDrivetrain.getYSpeed();
    
        String pathCordString = time + "," + robotX + "," + robotY + "," + rotation + "," + actualRotation + "," + xSpeed + "," + ySpeed;
    
        try {
          FileWriter myWriter = new FileWriter("/U/" + fileTime + ".txt", true);
          BufferedWriter bw = new BufferedWriter(myWriter);
          bw.write(pathCordString);
          bw.newLine();
          bw.close();
    
          //System.out.println("Successfully wrote to the file.");
        } catch (IOException e) {
          //System.out.println("An error occurred.");
          e.printStackTrace();
        }

        swerveDrivetrain.updateOdometry();
      }
      // SmartDashboard.putNumber("pos", swerveDrivetrain.m_frontLeft.getState().angle.getDegrees());
  }

    //IN TESTING
    public Command getAutoCommandImproved() {

      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Drivetrain.kMaxSpeed/4, 5);

      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(1, 0)),
      new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
      trajectoryConfig);

      PIDController xController = new PIDController(0.000001, 0, 0);
      PIDController yController = new PIDController(0.000001, 0, 0);
      TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Drivetrain.kMaxAngularSpeed, Math.PI / 4);
      ProfiledPIDController thetaController = new ProfiledPIDController(3, 0, 0, kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveDrivetrain::getPose, swerveDrivetrain.m_kinematics, xController, yController, thetaController, swerveDrivetrain::setModuleStates, swerveDrivetrain);

      return new SequentialCommandGroup(
        new InstantCommand(() -> swerveDrivetrain.resetOdometryNew(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerveDrivetrain.stopAllModules())
        );
    }

}
