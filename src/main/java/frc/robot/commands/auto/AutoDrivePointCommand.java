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