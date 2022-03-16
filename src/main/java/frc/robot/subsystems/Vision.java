package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.limelight.LimeLight;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.shooter.SpeedAngle;
import frc.robot.subsystems.shooter.TargetMapper;

public class Vision extends SubsystemBase {

    private LimeLight limelight;
    private boolean trackingMode = false;

    public Vision() {
        limelight = new LimeLight();
        limelight.setPipeline(0);

        // Disable Tracking
        // disableTracking();
    }

    @Override
    public void periodic() {
        double distance = getDistanceFromTargetInInches();
        SpeedAngle temp = TargetMapper.getSpeedAngleByDistance(distance);

        SmartDashboard.putBoolean("vision_enabled", isTrackingEnabled());
        SmartDashboard.putBoolean("vision_targetFound", isTargetFound());
        SmartDashboard.putNumber("vision_Xangle", getXAngleToTarget());
        SmartDashboard.putNumber("vision_Yangle", getYAngleToTarget());
        SmartDashboard.putNumber("vision_distance", distance);

        // DO NOT COMMMENT OUT OR REMOVE THESE TWO LINES BELOW, USED BY Shooter commands
        // See RobotContainer.configureSubsystemControllerBindings()
        SmartDashboard.putNumber("vision_hood_angle", temp.getAngle());
        SmartDashboard.putNumber("vision_shooter_speed", temp.getSpeed());
    }

    public double getDistanceFromTargetInInches() {
        if (!isTrackingEnabled()) {
            enableTracking();
        }

        double heightDiff = RobotConstants.TARGET_HEIGHTFROMGROUND - RobotConstants.CAMERA_HEIGHTFROMGROUND;
        double distance = 0;

        if (isTargetFound()) {
            double yAngleToTarget = limelight.getdegVerticalToTarget(); // degrees
            distance = heightDiff
                    / Math.tan((RobotConstants.CAMERA_ANGLEFROMPARALLEL + yAngleToTarget) * Math.PI / 180.0); // meters
                                                                                                              // now
                                                                                                              // inches
        }

        return distance;
    }

    public double getXAngleToTarget() {
        return limelight.getdegRotationToTarget();
    }

    public double getYAngleToTarget() {
        return limelight.getdegVerticalToTarget();
    }

    public void enableTracking() {
        // //System.out.println("ENABLED TRACKING");
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        trackingMode = true;
    }

    public void disableTracking() {
        // //System.out.println("DISABLED TRACKING");
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
