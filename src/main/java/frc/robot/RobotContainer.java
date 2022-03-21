// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.AutoDriveCommand;
import frc.robot.commands.auto.PlotPathCommand;
import frc.robot.commands.climber.ClimberLeftCommand;
import frc.robot.commands.drivetrain.FieldRelative;
import frc.robot.commands.drivetrain.StopDrivetrainCommand;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.shooter.FeederForward;
import frc.robot.commands.shooter.FeederReverse;
import frc.robot.commands.shooter.FeederStop;
import frc.robot.commands.shooter.HoodSetAngle;
import frc.robot.commands.shooter.HoodStop;
import frc.robot.commands.shooter.ShooterWheelsReverse;
import frc.robot.commands.shooter.ShooterWheelsSetSpeed;
import frc.robot.commands.shooter.ShooterWheelsStop;
import frc.robot.commands.vision.TurnToTarget;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.climber.ClimberLeftSubsystem;
import frc.robot.subsystems.climber.ClimberRightSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
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
  // public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public final ClimberLeftSubsystem climberLeftSubsystem = new ClimberLeftSubsystem();
  public final ClimberRightSubsystem climberRightSubsystem = new ClimberRightSubsystem();

  public final Vision vision = new Vision();

  public static final XboxController driveController = new XboxController(RobotConstants.DRIVE_CONTROLLER_PORT);
  public static final XboxController subsystemController = new XboxController(RobotConstants.SUBSYSTEM_CONTROLLER_PORT);

  public boolean isHoodUp = false;

  // Autonomous path generation related
  public static SendableChooser<String> autoPathChooser = new SendableChooser<>();
  public static SendableChooser<String> colorChooser = new SendableChooser<>();
  public ArrayList<DataPoint> testAuto;
  public ArrayList<DataPoint> runItBack;
  public ArrayList<DataPoint> turning;

  //For Long Auto
  public ArrayList<DataPoint> longAuto1; 
  public ArrayList<DataPoint> longAuto2; 


  public static boolean isPlotting = false;

  private Long startTime;
  private String fileTime;
  private DateTimeFormatter filenameFormatter;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //readPaths();
    setupPathChooser();

    SmartDashboard.putNumber("manual_hood_angle", 0.1);
    SmartDashboard.putNumber("manual_shooter_speed", 0);
    SmartDashboard.putNumber("kP", 0);
    SmartDashboard.putNumber("kI", 0);
    SmartDashboard.putNumber("kD", 0);
    SmartDashboard.putNumber("kF", 0.0475);
    SmartDashboard.putNumber("kAccel", 50000);
    SmartDashboard.putNumber("kVel", 25000);

    colorChooser.addOption("Red", "Red");
    colorChooser.addOption("Blue", "Blue");
    SmartDashboard.putData("Game Color", colorChooser);


    SmartDashboard.putString("DistanceTwoBall", "108");
    SmartDashboard.putString("DistanceManual", "75");


    // Configure the button bindings
    configureDriveControllerBindings();
    configureSubsystemControllerBindings();
  }

  public void readPaths() {
    // testAuto = readAutoFile("/U/testAuto2.txt");
    runItBack = readAutoFile("/files/runItBack.txt");
    // longAuto1 = readAutoFile("/U/LongAuto1.txt");
    // longAuto2 = readAutoFile("/U/LongAuto2.txt");
    // turning = readAutoFile("/U/turning.txt");
  }

  public ArrayList<DataPoint> readAutoFile(String filePath) {

    ArrayList<DataPoint> dataPathPoints = new ArrayList<>();
    File file = new File(filePath);

    try {
        Scanner myReader = new Scanner(file);

        while (myReader.hasNextLine()) {
            String data = myReader.nextLine();
            //List<String> dataPoint = Arrays.asList(data.split(","));
            String[] dataPointStr = data.split(",");
            long timeStamp = Long.parseLong(dataPointStr[0]);
            double xPos = Double.parseDouble(dataPointStr[1]);
            double yPos = Double.parseDouble(dataPointStr[2]);
            double angle = Double.parseDouble(dataPointStr[3]);
            double actRot = Double.parseDouble(dataPointStr[4]);
            double xS = Double.parseDouble(dataPointStr[5]);
            double yS = Double.parseDouble(dataPointStr[6]);

            DataPoint dataPoint = new DataPoint(timeStamp, xPos, yPos, angle, actRot, xS, yS);
            dataPathPoints.add(dataPoint);
        }

        myReader.close();
    } catch (FileNotFoundException e) {
        //System.out.println("An error occurred.");
        e.printStackTrace();
    }

    return dataPathPoints;
}

public void setupPathChooser() {
    String[] autoNames = {"testAuto", "JustAutoWTurn", "Turns and Shoots", "Run It Back", "Long Auto", "Hood Down"};

    for (String pathName : autoNames) {
      autoPathChooser.addOption(pathName, pathName);
    }

    SmartDashboard.putData("Auto Path", autoPathChooser);
  }

  public Command getAutonomousCommand() {

    String autoPath = autoPathChooser.getSelected();

    if (autoPath.equals("testAuto")) {

      return new SequentialCommandGroup(new ParallelCommandGroup(
        new AutoDriveCommand(testAuto, swerveDrivetrain, fieldRelative), new IntakeIn(intakeSubsystem)), 
        new ParallelCommandGroup(
          new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
          new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0))), 
          new SequentialCommandGroup(new WaitCommand(5), new FeederForward(feederSubsystem)))
          );

      //return new AutoDriveCommand(testAuto, swerveDrivetrain, fieldRelative);
    }

    else if(autoPath.equals("Run It Back")) {
      //faster
/*
      return new SequentialCommandGroup(
        // new HoodSetAngle(subsystem, angleGetter)
        new ParallelDeadlineGroup(new WaitCommand(1), new HoodSetAngle(hoodSubsystem, () -> (0.5))),
        new ParallelDeadlineGroup(
            new WaitCommand(1.5),
            new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
            new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0))), 
            new SequentialCommandGroup(new WaitCommand(0.5), new FeederForward(feederSubsystem))),
        new ParallelDeadlineGroup(new AutoDriveCommand(runItBack, swerveDrivetrain, fieldRelative), new IntakeIn(intakeSubsystem), 
        new FeederStop(feederSubsystem),
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem)), 
        new TurnToTarget(vision, swerveDrivetrain), new ParallelDeadlineGroup(new WaitCommand(0.5), new StopDrivetrainCommand(swerveDrivetrain)),

        new SequentialCommandGroup(new ParallelDeadlineGroup(new WaitCommand(0.3), new ParallelCommandGroup(
          new FeederReverse(feederSubsystem),
          new ShooterWheelsReverse(shooterWheelsSubsystem)
        ))), 
          new ParallelCommandGroup(
            new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
            new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0))), 
            new SequentialCommandGroup(new WaitCommand(0.7), new FeederForward(feederSubsystem))),
          
        new WaitCommand(0.3),
        new FeederStop(feederSubsystem),
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem),
        new IntakeStop(intakeSubsystem)
      );
      */
      return new SequentialCommandGroup(
        // new HoodSetAngle(subsystem, angleGetter)
        new ParallelDeadlineGroup(new WaitCommand(1), new HoodSetAngle(hoodSubsystem, () -> (0.6))),
        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
            new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0))), 
            new SequentialCommandGroup(new WaitCommand(0.5), new FeederForward(feederSubsystem))),
        new ParallelDeadlineGroup(new AutoDriveCommand(runItBack, swerveDrivetrain, fieldRelative), new IntakeIn(intakeSubsystem), 
        new FeederStop(feederSubsystem),
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem)), 
        new TurnToTarget(vision, swerveDrivetrain), new ParallelDeadlineGroup(new WaitCommand(0.5), new StopDrivetrainCommand(swerveDrivetrain)),
        new TurnToTarget(vision, swerveDrivetrain), new ParallelDeadlineGroup(new WaitCommand(0.4), new StopDrivetrainCommand(swerveDrivetrain)),
        new SequentialCommandGroup(new ParallelDeadlineGroup(new WaitCommand(0.3), new ParallelCommandGroup(
          new FeederReverse(feederSubsystem),
          new ShooterWheelsReverse(shooterWheelsSubsystem)
        ))), 
          new ParallelCommandGroup(
            new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
            new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0))), 
            new SequentialCommandGroup(new WaitCommand(1), new FeederForward(feederSubsystem))),
        new WaitCommand(1),
        new FeederStop(feederSubsystem),
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem)
      );
    }

    else if (autoPath.equals("JustAutoWTurn")) {
      return new SequentialCommandGroup(new AutoDriveCommand(runItBack, swerveDrivetrain, fieldRelative), new TurnToTarget(vision, swerveDrivetrain));
      
      //return new AutoDriveCommand(runItBack, swerveDrivetrain, fieldRelative);
    }

    else if(autoPath.equals("Hood Down")) {
      return new SequentialCommandGroup(new HoodSetAngle(hoodSubsystem, () -> (0.5)));

    }

    else if (autoPath.equals("Turns and Shoots")) {
      return new SequentialCommandGroup(new TurnToTarget(vision, swerveDrivetrain), new ParallelDeadlineGroup(new WaitCommand(1), new StopDrivetrainCommand(swerveDrivetrain)),
      
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
    else if (autoPath.equals("Long Auto")) {
      return new SequentialCommandGroup(

        new AutoDriveCommand(longAuto1, swerveDrivetrain, fieldRelative),
        new TurnToTarget(vision, swerveDrivetrain),
        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new StopDrivetrainCommand(swerveDrivetrain)),
        new ParallelDeadlineGroup(
          new WaitCommand(4),
          new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
          new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0))), 
          new SequentialCommandGroup(new WaitCommand(2), new FeederForward(feederSubsystem))),
        new FeederStop(feederSubsystem),
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem),
        new ParallelDeadlineGroup(
          new AutoDriveCommand(longAuto2, swerveDrivetrain, fieldRelative), new IntakeIn(intakeSubsystem)),
        new TurnToTarget(vision, swerveDrivetrain),
        new ParallelDeadlineGroup(
          new WaitCommand(1), new StopDrivetrainCommand(swerveDrivetrain)),
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new ParallelCommandGroup(
              new FeederReverse(feederSubsystem),
              new ShooterWheelsReverse(shooterWheelsSubsystem)))), 
        new ParallelCommandGroup(
          new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
          new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0))), 
          new SequentialCommandGroup(new WaitCommand(3), new FeederForward(feederSubsystem)))
      );
    }
    else {
      return new SequentialCommandGroup(new Command[] {});
    }
  }

  private void configureDriveControllerBindings() {

    // 'X' button to aim, bind to cmd TurnToTarget
    new JoystickButton(driveController, Button.kX.value)
        .whenPressed(new TurnToTarget(vision, swerveDrivetrain));

    // right bumper => intake in
    new JoystickButton(driveController, Button.kRightBumper.value)
        .whenPressed(new IntakeIn(intakeSubsystem))
        .whenReleased(new IntakeStop(intakeSubsystem));

    // left bumper => intake out
    new JoystickButton(driveController, Button.kLeftBumper.value)
        .whenPressed(new IntakeOut(intakeSubsystem))
        .whenReleased(new IntakeStop(intakeSubsystem));

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

  private void configureSubsystemControllerBindings() {

    // right bumper => shoot
    new JoystickButton(subsystemController, Button.kRightBumper.value)
      .whenPressed(new ParallelCommandGroup(
        new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("vision_hood_angle", 0.1))),
        new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0)))
      ))
      .whenReleased(new ParallelCommandGroup(
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem)
      ));

    new JoystickButton(subsystemController, Button.kLeftBumper.value)
      .whenPressed(new ParallelCommandGroup(
        new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("manual_hood_angle", 0.3
        ))),
        new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("manual_shooter_speed", 12750)))
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
        .whenPressed(new FeederForward(feederSubsystem))
        .whenReleased(new FeederStop(feederSubsystem));

    //#region PID Tuning
    // for PID tuning      
    // new JoystickButton(subsystemController, Button.kA.value)
    //     .whenPressed(new InstantCommand(()-> {
    //       swerveDrivetrain.turningMotorM1.config_kP(0, SmartDashboard.getNumber("kP", 0));
    //       swerveDrivetrain.turningMotorM1.config_kI(0, SmartDashboard.getNumber("kI", 0));
    //       swerveDrivetrain.turningMotorM1.config_kD(0, SmartDashboard.getNumber("kD", 0));
    //       swerveDrivetrain.turningMotorM1.config_kF(0, SmartDashboard.getNumber("kF", 0.0475));
    //       swerveDrivetrain.turningMotorM1.configMotionAcceleration(SmartDashboard.getNumber("kAccel", 100000));
    //       swerveDrivetrain.turningMotorM1.configMotionCruiseVelocity(SmartDashboard.getNumber("kVel", 50000));
    //       swerveDrivetrain.turningMotorM2.config_kP(0, SmartDashboard.getNumber("kP", 0));
    //       swerveDrivetrain.turningMotorM2.config_kI(0, SmartDashboard.getNumber("kI", 0));
    //       swerveDrivetrain.turningMotorM2.config_kD(0, SmartDashboard.getNumber("kD", 0));
    //       swerveDrivetrain.turningMotorM2.config_kF(0, SmartDashboard.getNumber("kF", 0.0475));
    //       swerveDrivetrain.turningMotorM2.configMotionAcceleration(SmartDashboard.getNumber("kAccel", 100000));
    //       swerveDrivetrain.turningMotorM2.configMotionCruiseVelocity(SmartDashboard.getNumber("kVel", 50000));
    //       swerveDrivetrain.turningMotorM3.config_kP(0, SmartDashboard.getNumber("kP", 0));
    //       swerveDrivetrain.turningMotorM3.config_kI(0, SmartDashboard.getNumber("kI", 0));
    //       swerveDrivetrain.turningMotorM3.config_kD(0, SmartDashboard.getNumber("kD", 0));
    //       swerveDrivetrain.turningMotorM3.config_kF(0, SmartDashboard.getNumber("kF", 0.0475));
    //       swerveDrivetrain.turningMotorM3.configMotionAcceleration(SmartDashboard.getNumber("kAccel", 100000));
    //       swerveDrivetrain.turningMotorM3.configMotionCruiseVelocity(SmartDashboard.getNumber("kVel", 50000));
    //       swerveDrivetrain.turningMotorM4.config_kP(0, SmartDashboard.getNumber("kP", 0));
    //       swerveDrivetrain.turningMotorM4.config_kI(0, SmartDashboard.getNumber("kI", 0));
    //       swerveDrivetrain.turningMotorM4.config_kD(0, SmartDashboard.getNumber("kD", 0));
    //       swerveDrivetrain.turningMotorM4.config_kF(0, SmartDashboard.getNumber("kF", 0.0475));
    //       swerveDrivetrain.turningMotorM4.configMotionAcceleration(SmartDashboard.getNumber("kAccel", 100000));
    //       swerveDrivetrain.turningMotorM4.configMotionCruiseVelocity(SmartDashboard.getNumber("kVel", 50000));
    //     }));

    //#endregion

    // new JoystickButton(subsystemController, Button.kB.value)
    //   .whenPressed(new InstantCommand(() -> swerveDrivetrain.m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI / 2)))))
    //   .whenReleased(new InstantCommand(() -> swerveDrivetrain.m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)))));

    new JoystickButton(subsystemController, Axis.kLeftTrigger.value)
        .whenPressed(new InstantCommand(()->shooterWheelsSubsystem.set(subsystemController.getLeftTriggerAxis()), shooterWheelsSubsystem))
        .whenReleased(new InstantCommand(shooterWheelsSubsystem::stopShooter, shooterWheelsSubsystem));
        
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
        SmartDashboard.putBoolean("Is Plotting", true);
      }
      else{
        //System.out.println("Stops plotting");
        SmartDashboard.putBoolean("Is Plotting", false);
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

}
