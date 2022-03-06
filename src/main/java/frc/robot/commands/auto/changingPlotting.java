package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class changingPlotting extends CommandBase{
    private boolean isPlotting;


    public changingPlotting(boolean isPlotting) {
        this.isPlotting = isPlotting;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
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
