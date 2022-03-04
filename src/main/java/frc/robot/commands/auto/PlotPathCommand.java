package frc.robot.commands.auto;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class PlotPathCommand extends CommandBase {
    private Long startTime;
    private String fileTime;
    private DateTimeFormatter filenameFormatter;
    public Drivetrain swerveDrive;

    public PlotPathCommand (Drivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;

        // toogle plotting
        if (!RobotContainer.isPlotting) {
            RobotContainer.isPlotting = true;
            fileCreator();
        } else {
            RobotContainer.isPlotting = false;
        }

        addRequirements((Subsystem) swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.isPlotting) {
            drivingPlotter();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (!RobotContainer.isPlotting);
    }

    private void fileCreator() {
        filenameFormatter = DateTimeFormatter.ofPattern("dd-MM-yyyy-HH-mm-ss");

        //Starts and Stops plotting
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

    public void drivingPlotter() {
        Long time = System.currentTimeMillis() - startTime;
        double robotX = swerveDrive.poseEstimator().getEstimatedPosition().getX();
        double robotY = swerveDrive.poseEstimator().getEstimatedPosition().getY();
        double rotation = swerveDrive.poseEstimator().getEstimatedPosition().getRotation().getDegrees();
        double actualRotation = swerveDrive.getRot();
        double xSpeed = swerveDrive.getXSpeed();
        double ySpeed = swerveDrive.getYSpeed();
      
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

        swerveDrive.updateOdometry();
    }

}