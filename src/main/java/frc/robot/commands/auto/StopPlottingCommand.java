package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class StopPlottingCommand extends CommandBase{
    private final Drivetrain swerveDrive;

    public StopPlottingCommand(Drivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerveDrive.isPlotting = false; //should interrupt PlotPathCommand
    }

    // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
    }

    // Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}

}
