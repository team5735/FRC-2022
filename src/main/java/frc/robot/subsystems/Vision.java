package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.limelight.LimeLight;
import frc.robot.constants.RobotConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    private LimeLight limelight;
    private boolean trackingMode = false;
	private boolean hasValidTarget = false;

    public Vision() {
        limelight = new LimeLight();
        limelight.setPipeline(0);

        //Disable Tracking
    }

    public void periodic() {
        hasValidTarget = isTargetFound(); //set it to the isTargetFound
    }

    public double getDistanceFromTarget() {
    
        if(!isTrackingEnabled()) {
            enableTracking();
        }

        boolean foundDistance = false;

        double heightDiff = RobotConstants.TARGET_HEIGHTFROMGROUND - RobotConstants.CAMERA_HEIGHTFROMGROUND;
		double distance = 0;

		// while (!foundDistance) {
			// double yAngleToTarget = Units.degreesToRadians(limelight.getdegVerticalToTarget()); // radians
			double yAngleToTarget = limelight.getdegVerticalToTarget(); // degrees
			distance = /*heightDiff*/ 6.5 / Math.tan((RobotConstants.CAMERA_ANGLEFROMPARALLEL - yAngleToTarget)*Math.PI/180); // meters now inches
			if (distance > 1) {
				foundDistance = true;
			}
        // }
        return distance;

    }

	public double getYAngleToTarget() {
		return limelight.getdegVerticalToTarget();
	}


    public void enableTracking() {
		System.out.println("ENABLED TRACKING");
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
		trackingMode = true;
	}

	public void disableTracking() {
		System.out.println("DISABLED TRACKING");
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
		trackingMode = false;
	}
	
	public boolean isTrackingEnabled() {
		return trackingMode;
	}

	public boolean isTargetFound() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) > 0;
	}

	public boolean hasValidTarget() {
		SmartDashboard.putBoolean("Has Target", hasValidTarget);
		return hasValidTarget;
	}

	public LimeLight getLimelight() {
		return limelight;
	}

	public double getTX() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
	}

	public double getTY() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
	}

}
