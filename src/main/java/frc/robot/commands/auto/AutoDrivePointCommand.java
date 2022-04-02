package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.plotter.DataPoint;

public class AutoDrivePointCommand extends CommandBase{
    
    private ArrayList<DataPoint> currentPath;
    private int currentIndex = 1;
    private Drivetrain swerveDrive;
    private boolean fieldRelative;
    public static boolean commandFinished = false;

    private double radiusThreshold = 0.0001;

    public AutoDrivePointCommand(ArrayList<DataPoint> currentPath, Drivetrain swerveDrive, boolean fieldRelative) {
        this.currentPath = currentPath;
        this.swerveDrive = swerveDrive;
        this.fieldRelative = fieldRelative;

        addRequirements((Subsystem) swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        /*
        Get the X and Y pos for the robot and the current rotation

        get the next point in the array and try and drive to that location

        */

        if(currentIndex >= currentPath.size()) {
            commandFinished = true;
            return;
        }
        

        double actRot = currentPath.get(currentIndex).actualRot;
        Long time = currentPath.get(currentIndex).timeStamp;
        Long prevTime = currentPath.get(currentIndex-1).timeStamp;
        long deltaTime = time-prevTime;

        double currentX = swerveDrive.poseEstimator().getEstimatedPosition().getX();
        double currentY = swerveDrive.poseEstimator().getEstimatedPosition().getY();

        double xPos = currentPath.get(currentIndex).x;
        double yPos = currentPath.get(currentIndex).y;

        double dx = currentX - xPos;
        double dy = currentY - yPos;

        double xSpeed = dx / deltaTime;
        double ySpeed = dy / deltaTime;
        
        //System.out.println(xSpeed + ", " + ySpeed + ", " + actRot);
        swerveDrive.drive(xSpeed, ySpeed, actRot, fieldRelative);

        swerveDrive.updateOdometry();

        if(Math.sqrt((dy*dy) + (dx*dx)) < radiusThreshold) {

            currentIndex++;

        }

        try {
            Thread.sleep(time - prevTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return commandFinished;
    }
}