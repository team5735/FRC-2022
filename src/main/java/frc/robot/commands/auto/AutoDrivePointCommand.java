package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.plotter.DataPoint;

public class AutoDrivePointCommand extends CommandBase{
    
    private ArrayList<DataPoint> currentPath;
    private Drivetrain swerveDrive;
    private boolean fieldRelative;
    public static boolean commandFinished = false;

    private double radiusThreshold = 10;

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

        if(!commandFinished) {

            for (int i = 1; i < currentPath.size(); i++) {

                boolean hasReachedPoint = false;
                double xSpeed = currentPath.get(i).xSpeed;
                double ySpeed = currentPath.get(i).ySpeed;
                double actRot = currentPath.get(i).actualRot;
                Long time = currentPath.get(i).timeStamp;
                Long prevTime = currentPath.get(i-1).timeStamp;

                while(!hasReachedPoint) {

                    double currentX = swerveDrive.poseEstimator().getEstimatedPosition().getX();
                    double currentY = swerveDrive.poseEstimator().getEstimatedPosition().getY();

                    double xPos = currentPath.get(i).x;
                    double yPos = currentPath.get(i).y;

                    double x = currentX - xPos;
                    double y = currentY - yPos;

                    if(Math.sqrt((y*y) + (x*x)) < radiusThreshold) {

                        hasReachedPoint = true;

                    }

                    else {

                        System.out.println(xSpeed + ", " + ySpeed + ", " + actRot);
                        swerveDrive.drive(xSpeed, ySpeed, actRot, fieldRelative);

                    }


                }

                try {
                    Thread.sleep(time - prevTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
    
                
            }
        }



    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}