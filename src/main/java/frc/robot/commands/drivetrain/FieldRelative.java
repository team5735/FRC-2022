package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class FieldRelative extends CommandBase{
    
    private boolean fieldRelative;
    private Drivetrain swerveDrive;

    public FieldRelative(Drivetrain swerveDrive, boolean fieldRelative) {
        this.swerveDrive = swerveDrive;
        this.fieldRelative = fieldRelative;

        addRequirements((Subsystem) swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        swerveDrive.resetAHRS();
        //fieldRelative = !fieldRelative;

        RobotContainer.fieldRelative = fieldRelative;
        SmartDashboard.putBoolean("fieldRelative", fieldRelative);
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

    public boolean getFieldRelative() {
        return fieldRelative;
    }

}
