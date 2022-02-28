package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;

public class TurnToTarget extends CommandBase {

    private final Drivetrain drivetrain;
    private final Vision vision;
    
    public TurnToTarget(Vision vision, Drivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        addRequirements((Subsystem) drivetrain);
        // addRequirements(vision);
    }

    // Called when the command is initially scheduled.
	@Override
	public void initialize() {
		vision.enableTracking();
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
        double steering_adjust;
        double Kp = -0.1;  // Proportional control constant
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<tx>").getDouble(0);

        // Make sure Vision Tracking is Always Running
        if(!vision.isTrackingEnabled()) vision.enableTracking();
        // ADD DEGREE ERROR?
        // SPIN UNTIL HAS VALID TARGET
        // HOW TO MAKE ROBOT SPIN
        // HAS VALID TARGET SAME AS FETCHING tv

        if(vision.hasValidTarget()) {
            // drivetrain.drive
            drivetrain.drive(0,0,5, false);
        }
        else {
            if(tx > 1.0) {
                steering_adjust = Kp * tx;
                drivetrain.drive(0,0,steering_adjust, false);
            }
        }
        // Also want to spin more until the target is closer to the center
	}
	
	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
        System.out.println("TURN TARGET COMMAND | END");
		SmartDashboard.putNumber("DISTANCE", vision.getDistanceFromTarget());
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}

/*
IN ORDER TO RETRIEVE VALUES:
NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);

IN ORDER TO SET VALUES:
NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(<value>);

    tv	    Whether the limelight has any valid targets (0 or 1)
    tx	    Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    ty	    Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    ta	    Target Area (0% of image to 100% of image)
    ts	    Skew or rotation (-90 degrees to 0 degrees)
    tl	    The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
    tshort	Sidelength of shortest side of the fitted bounding box (pixels)
    tlong	Sidelength of longest side of the fitted bounding box (pixels)
    thor	Horizontal sidelength of the rough bounding box (0 - 320 pixels)
    tvert	Vertical sidelength of the rough bounding box (0 - 320 pixels)
    getpipe	True active pipeline index of the camera (0 .. 9)
    camtran	Results of a 3D position solution, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)

PID Stands For Proportional Integral Derivative
    kp      

*/

/* UNRELATED CODE FRAMEWORK FOR MYSELF
package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limeLightReader extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);
}
*/