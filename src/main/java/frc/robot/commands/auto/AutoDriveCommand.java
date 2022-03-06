package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.plotter.DataPoint;

public class AutoDriveCommand extends CommandBase {
    
    private ArrayList<DataPoint> currentPath;
    private Drivetrain swerveDrive;
    private boolean fieldRelative;
    private boolean commandFinished = false;

    public AutoDriveCommand(ArrayList<DataPoint> currentPath, Drivetrain swerveDrive, boolean fieldRelative) {
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
        if (!commandFinished) {
            for (int i = 1; i < currentPath.size(); i++) {

                double xSpeed = currentPath.get(i).xSpeed;
                double ySpeed = currentPath.get(i).ySpeed;
                double actRot = currentPath.get(i).actualRot;
                Long time = currentPath.get(i).timeStamp;
                Long prevTime = currentPath.get(i-1).timeStamp;

                System.out.println(time + ", " + xSpeed + ", " + ySpeed);
                swerveDrive.drive(xSpeed, ySpeed, actRot, fieldRelative);

                try {
                    Thread.sleep(time - prevTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                // double xDistance = currentPath.get(i).x - currentPath.get(i - 1).x;
                // double yDistance = currentPath.get(i).y - currentPath.get(i - 1).y;
                // double time = (double)(currentPath.get(i).timeStamp - currentPath.get(i - 1).timeStamp);
                // double xSpeed = xDistance / time * 100;
                // double ySpeed = yDistance / time * 100;
                // double rotSpeed = (currentPath.get(i).degrees - currentPath.get(i-1).degrees)/time;

                // // boolean xGreater = false;
                // // boolean yGreater = false;

                // // if(currentPath.get(i).x > currentPath.get(i - 1).x) {
                // //     xGreater = true;
                // // }
                // // if(currentPath.get(i).y > currentPath.get(i - 1).y) {
                // //     yGreater = true;
                // // }

                // double distanceThreshold = 0.1;

                // boolean xSpeedStop = false;
                // boolean ySpeedStop = false;

                // while(!xSpeedStop || !ySpeedStop) {
                //     // System.out.print("deltaX: " + Math.abs(swerveDrive.poseEstimator().getEstimatedPosition().getX() - currentPath.get(i).x));
                //     // System.out.println("deltaY: " + Math.abs(swerveDrive.poseEstimator().getEstimatedPosition().getY() - currentPath.get(i).y));
                //     System.out.println("i: " + i + "  xSpeed: " + xSpeed + ",  ySpeed: " + ySpeed);
                //     swerveDrive.drive(xSpeed, ySpeed, 0.0, fieldRelative);
                //     swerveDrive.updateOdometry();
                //     if(Math.abs(swerveDrive.poseEstimator().getEstimatedPosition().getX() - currentPath.get(i).x) < distanceThreshold) {
                //         xSpeedStop = true;
                //         xSpeed = 0.0;
                //     }
                //     if(Math.abs(swerveDrive.poseEstimator().getEstimatedPosition().getY() - currentPath.get(i).y) < distanceThreshold) {
                //         ySpeedStop = true;
                //         ySpeed = 0.0;
                //     }
                // }
                
                //swerveDrive.drive(xS, yS, 0.0, fieldRelative);
            }
        }

        commandFinished = true;
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

    public void endedCommand() {
        commandFinished = true;
    }

}
