package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;

public class StopDrivetrainCommand extends CommandBase{

    Drivetrain swerveDrive;

    public StopDrivetrainCommand(Drivetrain swerveDrive) {

        this.swerveDrive = swerveDrive;

        addRequirements((Subsystem) swerveDrive);
    }

    // Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

    // Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

        swerveDrive.drive(0, 0, 0, false);
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

    public double deadband(double input, double deadband) {
        if (Math.abs(input) < deadband) return 0;
        return input;
    }
    
}
