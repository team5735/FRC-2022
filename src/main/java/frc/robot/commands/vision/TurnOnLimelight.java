package frc.robot.commands.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class TurnOnLimelight extends CommandBase {
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Vision vision;
	
	public TurnOnLimelight(Vision vision) {
        this.vision = vision;
		// Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(vision);
	}
	
	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
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
