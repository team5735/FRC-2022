package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class TurnToTarget extends CommandBase {

    private final Drivetrain drivetrain;
    private final Vision vision;
    private boolean isFinished = false;
    private int rotationCompleted = 0;
    private long lastRecordedTime; 
    private long startTime;

    PIDController pid = new PIDController(0.08, 0.15, 0);
    
    public TurnToTarget(Vision vision, Drivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        addRequirements((Subsystem) drivetrain);
        addRequirements((Subsystem) vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rotationCompleted = 0;
        startTime = System.currentTimeMillis();
        lastRecordedTime = System.currentTimeMillis();
        pid.reset();
    }
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
        double steering_adjust;
        double Kp = -0.15;  // Proportional control constant
        isFinished = false;
 
        // check if command runs too long
        if (System.currentTimeMillis() - startTime > ShooterConstants.TURN_TIMEOUT) {
            isFinished = true;
            return;
        }

        // check if pid runs too long
        if (System.currentTimeMillis() - lastRecordedTime > ShooterConstants.MIN_TURN_TIME) {
            isFinished = true;
            return;
        }

        // Make sure Vision Tracking is running
        if (!vision.isTrackingEnabled())
            vision.enableTracking();

        // ADD DEGREE ERROR?
        // SPIN UNTIL HAS VALID TARGET
        // HOW TO MAKE ROBOT SPIN
        // HAS VALID TARGET SAME AS FETCHING tv
        if (!vision.isTargetFound()) {
            // drivetrain.drive
            drivetrain.drive(0, 0, 3.5, false);
            // rotationCompleted += 3;
            lastRecordedTime = System.currentTimeMillis();
        } else {
            double tx = vision.getTX();

            if (Math.abs(tx) > 3) {
                // Also want to spin more until the target is closer to the center
                steering_adjust = pid.calculate(tx, 0);
                drivetrain.drive(0, 0, steering_adjust, false);
            } else {
                isFinished = true;
            }
        }

        //System.out.println("turn_isFinished =" + isFinished);
        //System.out.println("turn_rotationCompleted = " + rotationCompleted);
	}
	
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        long now = System.currentTimeMillis() ;
        System.out.println("turn-to-target ends=" + now + ", took=" + (now - startTime) + ", pidTim=" + (now - lastRecordedTime));
        //System.out.println("TURN TARGET COMMAND | END: " + interrupted);
        //SmartDashboard.putBoolean("TURN TARGET COMMAND | END", interrupted);
	}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (isFinished);
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