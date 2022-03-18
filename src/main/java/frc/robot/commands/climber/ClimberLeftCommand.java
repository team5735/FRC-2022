package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberLeftSubsystem;

public class ClimberLeftCommand extends CommandBase{

    private ClimberLeftSubsystem climberSubsystem;
    private final double DEADBAND = 0.05;

    public ClimberLeftCommand(ClimberLeftSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements((Subsystem) climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double yInput = deadband(RobotContainer.subsystemController.getLeftY(), DEADBAND);
        double xInput = deadband(RobotContainer.subsystemController.getLeftX(), DEADBAND);
        if(yInput > 0) {
            climberSubsystem.set(yInput * ClimberConstants.WINCH_UP_SPEED);
        }
        else if(yInput < 0) {
            climberSubsystem.set(yInput * ClimberConstants.WINCH_DOWN_SPEED);
        } else {
            climberSubsystem.set(0);
        }

        climberSubsystem.rotate(xInput * ClimberConstants.ARM_ROTATE_SPEED);
    }
    
	@Override
	public void end(boolean interrupted) {
	}

    @Override
    public boolean isFinished() {
        return false;
    }

    public double deadband(double input, double deadband) {
        if (Math.abs(input) < deadband) return 0;
        return input;
    }

}
