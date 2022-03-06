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

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.commands.shooter.HoodStop;
import frc.robot.commands.shooter.ShooterWheelsSetSpeed;
import frc.robot.commands.shooter.ShooterWheelsStop;
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
public class RobotContainer {
  public static boolean fieldRelative = false;
  public final Drivetrain swerveDrivetrain = new Drivetrain();

  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final ShooterWheelsSubsystem shooterWheelsSubsystem = new ShooterWheelsSubsystem();
  public final HoodSubsystem hoodSubsystem = new HoodSubsystem();
  public final FeederSubsystem feederSubsystem = new FeederSubsystem();

  public final Vision vision = new Vision();

  public static final XboxController driveController = new XboxController(RobotConstants.DRIVE_CONTROLLER_PORT);
  public static final XboxController subsystemController = new XboxController(RobotConstants.SUBSYSTEM_CONTROLLER_PORT);

  public boolean isHoodUp = false;

  // Autonomous path generation related
  public static SendableChooser<String> autoPathChooser = new SendableChooser<>();
  public ArrayList<DataPoint> testAuto;
  public static boolean isPlotting = false;

  private Long startTime;
  private String fileTime;
  private DateTimeFormatter filenameFormatter;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    readPaths();
    setupPathChooser();

    SmartDashboard.putNumber("manual_hood_angle", 0.1);
    SmartDashboard.putNumber("manual_shooter_speed", 0);

    // Configure the button bindings
    configureDriveControllerBindings();
    configureSubsystemControllerBindings();
  }

  public void readPaths() {
    testAuto = readAutoFile("/U/testAuto2.txt");
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
        System.out.println("An error occurred.");
        e.printStackTrace();
    }

    return dataPathPoints;
}

public void setupPathChooser() {
    String[] autoNames = {"testAuto2"};

    for (String pathName : autoNames) {
      autoPathChooser.addOption(pathName, pathName);
    }

    SmartDashboard.putData("Auto Path", autoPathChooser);
  }

  public Command getAutonomousCommand() {

    String autoPath = autoPathChooser.getSelected();

    if (autoPath.equals("testAuto2")) {
      return new AutoDriveCommand(testAuto, swerveDrivetrain, fieldRelative);
    }
    else {
      return new SequentialCommandGroup(new Command[] {});
    }
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

    //Y button => toggle autonomous path plotting
    // new JoystickButton(driveController, Button.kY.value)
    //   .whenPressed(new PlotPathCommand(swerveDrivetrain, isPlotting));

  }

  public void configureAutoButton() {
    if(driveController.getYButtonPressed()){
      isPlotting = !isPlotting;
      fileCreator();
    }
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
        new HoodSetAngle(hoodSubsystem, () -> (SmartDashboard.getNumber("manual_hood_angle", 0.1))),
        new ShooterWheelsSetSpeed(shooterWheelsSubsystem, () -> (SmartDashboard.getNumber("vision_shooter_speed", 0)))
      ))
      .whenReleased(new ParallelCommandGroup(
        new HoodStop(hoodSubsystem),
        new ShooterWheelsStop(shooterWheelsSubsystem)
      ));

    //for PID tuning      
    new JoystickButton(subsystemController, Button.kA.value)
        .whenPressed(new InstantCommand(()->shooterWheelsSubsystem.set(11500), shooterWheelsSubsystem))
        .whenReleased(new InstantCommand(shooterWheelsSubsystem::stopShooter, shooterWheelsSubsystem));

    new JoystickButton(subsystemController, Button.kY.value)
        .whenPressed(new FeederForward(feederSubsystem))
        .whenReleased(new FeederStop(feederSubsystem));

    new JoystickButton(subsystemController, Button.kB.value)
      .whenPressed(new HoodSetAngle(hoodSubsystem, () -> (0.75)));

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

  private void fileCreator() {

    filenameFormatter = DateTimeFormatter.ofPattern("dd-MM-yyyy-HH-mm-ss");

    //Starts and Stops plotting
      if(isPlotting) {
        try {
          fileTime = filenameFormatter.format(LocalDateTime.now());
          File myObj = new File("/U/" + fileTime + ".txt");
          startTime = System.currentTimeMillis();
          if (myObj.createNewFile()) {
            System.out.println("File created: " + myObj.getName());
          } else {
            System.out.println("File already exists.");
          }
        
        } catch (IOException e) {
          System.out.println("An error occurred.");
          e.printStackTrace();
        }
        SmartDashboard.putBoolean("Is Plotting", true);
      }
      else{
        System.out.println("Stops plotting");
        SmartDashboard.putBoolean("Is Plotting", false);
      }
    }
    public void drivingPlotter() {
      if(isPlotting) {
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
    
          System.out.println("Successfully wrote to the file.");
        } catch (IOException e) {
          System.out.println("An error occurred.");
          e.printStackTrace();
        }

        swerveDrivetrain.updateOdometry();
      }
    }

}
