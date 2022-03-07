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
    public static boolean commandFinished = false;

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

                System.out.println(time + ", " + xSpeed + ", " + ySpeed + ", " + actRot);
                swerveDrive.drive(xSpeed, ySpeed, actRot, fieldRelative);

                try {
                    Thread.sleep(time - prevTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
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
