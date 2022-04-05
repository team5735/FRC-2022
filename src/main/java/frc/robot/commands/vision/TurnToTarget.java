package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnToTarget extends CommandBase {

    private final Drivetrain drivetrain;
    private final Vision vision;
    private boolean isFinished = false;
    private long startTime;

    PIDController pid = new PIDController(0.08, 0.15, 0);
    //PIDController pid = new PIDController(0.1, 0.04, 0);

    public TurnToTarget(Vision vision, Drivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        addRequirements((Subsystem) drivetrain);
        addRequirements((Subsystem) vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        pid.reset();
    }
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
        double steering_adjust = 0.0;
        isFinished = false;
 
        // check if command runs too long
        if (System.currentTimeMillis() - startTime > ShooterConstants.TURN_TIMEOUT) {
            isFinished = true;
            return;
        }

        // Make sure Vision Tracking is running
        if (!vision.isTrackingEnabled())
            vision.enableTracking();

        double tx = vision.getTX();

        // ADD DEGREE ERROR?
        // SPIN UNTIL HAS VALID TARGET
        // HOW TO MAKE ROBOT SPIN
        // HAS VALID TARGET SAME AS FETCHING tv
        if (!vision.isTargetFound()) {
            // drivetrain.drive
            drivetrain.drive(0, 0, 2.5, false);
        } else {
            // Also want to spin more until the target is closer to the center
            steering_adjust = pid.calculate(tx, 0);
            drivetrain.drive(0, 0, steering_adjust, false);
        }

        // long now = System.currentTimeMillis() ;
        // System.out.println("turn-to-target ends=" + now + ", took=" + (now - startTime) 
        //     + ", isFinished=" + isFinished + ", tx=" + vision.getTX() + ", steering_adjust=" + steering_adjust);
	}
	
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //System.out.println("TURN TARGET COMMAND | END: " + interrupted);
        //SmartDashboard.putBoolean("TURN TARGET COMMAND | END", interrupted);
	}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (isFinished);
	}

}
