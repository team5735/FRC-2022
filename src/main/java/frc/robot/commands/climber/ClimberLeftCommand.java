package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberLeftSubsystem;

public class ClimberLeftCommand extends CommandBase{

    private ClimberLeftSubsystem climberSubsystem;

    public ClimberLeftCommand(ClimberLeftSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements((Subsystem) climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double yInput = -RobotContainer.subsystemController.getLeftTriggerAxis() * ClimberConstants.WINCH_UP_SPEED;

        if(RobotContainer.subsystemController.getBButton()) {
            yInput=-yInput;
        }

        // double xInput = Math.pow(deadband(RobotContainer.subsystemController.getLeftX(), ClimberConstants.DEADBAND * 2), 3);
        double xInput = -deadband(RobotContainer.subsystemController.getLeftY(), ClimberConstants.DEADBAND) * ClimberConstants.ARM_ROTATE_UP_SPEED;

        /*
        if (RobotContainer.subsystemController.getPOV() != -1){
            System.out.println(RobotContainer.subsystemController.getPOV());
        }

        switch (RobotContainer.subsystemController.getPOV()) {
            case 0:
                xInput = ClimberConstants.ARM_ROTATE_UP_SPEED;
                break;
            case 90:
                xInput = ClimberConstants.ARM_ROTATE_UP_SPEED * 0.5;
                break;
            case 180:
                xInput = -ClimberConstants.ARM_ROTATE_DOWN_SPEED;
                break;
            case 270:
                xInput = -ClimberConstants.ARM_ROTATE_DOWN_SPEED * 0.5;
                break; 
            default:
                break;
        }
        */

        if(yInput > 0) { //ARMS DOWN / WINCH IN
            if(true || climberSubsystem.getEncoderValue() < ClimberConstants.LEFT_ENCODER_MIN || RobotContainer.subsystemController.getYButton()) {
                climberSubsystem.set(yInput * ClimberConstants.WINCH_UP_SPEED);
            } else {
                climberSubsystem.set(0);
            }
        }
        else if(yInput < 0) { //ARMS UP / WINCH OUT
            if(true || climberSubsystem.getEncoderValue() > ClimberConstants.LEFT_ENCODER_MAX || RobotContainer.subsystemController.getYButton()) {
                climberSubsystem.set(yInput * ClimberConstants.WINCH_DOWN_SPEED);
            } else {
                climberSubsystem.set(0);
            }
            // climberSubsystem.set(yInput * ClimberConstants.WINCH_DOWN_SPEED);
            
        } else {
            climberSubsystem.set(0);
        }


        if(xInput > 0) { 
            climberSubsystem.rotate(xInput * ClimberConstants.ARM_ROTATE_UP_SPEED);
        } else if(xInput < 0 /*&& climberSubsystem.getEncoderValue() > ClimberConstants.LEFT_ENCODER_MIN*/) {
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
