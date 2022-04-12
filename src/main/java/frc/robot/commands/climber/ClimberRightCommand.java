package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberRightSubsystem;

public class ClimberRightCommand extends CommandBase{

    private ClimberRightSubsystem climberSubsystem;

    public ClimberRightCommand(ClimberRightSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements((Subsystem) climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        double yInput = RobotContainer.subsystemController.getRightTriggerAxis() * ClimberConstants.WINCH_UP_SPEED;

        if(!RobotContainer.subsystemController.getBButton()) {
            yInput=-yInput;
        }

        // double xInput = Math.pow(deadband(RobotContainer.subsystemController.getLeftX(), ClimberConstants.DEADBAND * 2), 3);
        double xInput = deadband(RobotContainer.subsystemController.getRightY(), ClimberConstants.DEADBAND) * ClimberConstants.ARM_ROTATE_UP_SPEED;

        if(yInput > 0) {
            climberSubsystem.set(yInput * ClimberConstants.WINCH_UP_SPEED);
        }
        else if(yInput < 0) {
            climberSubsystem.set(yInput * ClimberConstants.WINCH_DOWN_SPEED);
        } else {
            climberSubsystem.set(0);
        }

        if(xInput > 0) { //UP
            climberSubsystem.rotate(xInput * ClimberConstants.ARM_ROTATE_UP_SPEED);
        } else if(xInput < 0) { //DOWN
            climberSubsystem.rotate(xInput * ClimberConstants.ARM_ROTATE_DOWN_SPEED);
        } else {
            climberSubsystem.rotate(0);
        }
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
