package frc.robot.commands.drivetrain;

import edu.wpi.first.cscore.CameraServerJNI.LoggerFunction;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;
import frc.robot.subsystems.Drivetrain;

public class DriveWithXboxController extends CommandBase {
    
    private Drivetrain swerveDrive;
    public boolean fieldRelative;
    private double deadbandConstant;
    private static double xSpeedValue, ySpeedValue, rotValue;
    
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public DriveWithXboxController(Drivetrain swerveDrive) {

        this.swerveDrive = swerveDrive;
        fieldRelative = RobotContainer.fieldRelative;

        // needs fine tuning but this is to replace the constant used for the deadband
        deadbandConstant = 0.05;

        addRequirements((Subsystem) swerveDrive);
    }

    // Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

    // Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
        final var xSpeed = -m_xspeedLimiter.calculate(deadband(Math.pow(RobotContainer.driveController.getLeftY(),3), deadbandConstant)) * frc.robot.subsystems.Drivetrain.kMaxSpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(deadband(Math.pow(RobotContainer.driveController.getLeftX(), 3), deadbandConstant)) * frc.robot.subsystems.Drivetrain.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = m_rotLimiter.calculate(RobotContainer.driveController.getLeftTriggerAxis() - RobotContainer.driveController.getRightTriggerAxis()) * frc.robot.subsystems.Drivetrain.kMaxAngularSpeed;
        
        if(LoggingConstants.DRIVING_LEVEL.ordinal() >= LoggingLevel.COMPETITION.ordinal()) {
            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("ySpeed", ySpeed);
            SmartDashboard.putNumber("rot", rot);
            SmartDashboard.putBoolean("fieldRelative", fieldRelative);
        }

        // m_swerve.drive(0, 0, rot, false);

        // Y butoon => toggle field centric
        if (RobotContainer.driveController.getYButtonPressed()) {
            swerveDrive.resetAHRS();
            swerveDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        }

        xSpeedValue = xSpeed;
        ySpeedValue = ySpeed;
        rotValue = rot;


        ChassisSpeeds chassisSpeeds;
        if(fieldRelative) {
            //Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, swerveDrive.getRotation2d());
        }
        else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        // COnvert Chassis speeds to individual module states
        SwerveModuleState[] moduleState = swerveDrive.m_kinematics.toSwerveModuleStates(chassisSpeeds);

        swerveDrive.setModuleStates(moduleState);

        //System.out.println(swerveDrive.getPose().getX() + ",    " + swerveDrive.getPose().getY() );

        swerveDrive.updateOdometry();

        //swerveDrive.drive(xSpeed, ySpeed, rot, false);
    }

    // Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}
	
	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

    public double deadband(double input, double deadband) {
        if (Math.abs(input) < deadband) return 0;
        return input;
    }

    public static double xSpeed() {
        return xSpeedValue;
    }
    public static double ySpeed() {
        return ySpeedValue;
    }
    public static double rotation() {
        return rotValue;
    }

}
