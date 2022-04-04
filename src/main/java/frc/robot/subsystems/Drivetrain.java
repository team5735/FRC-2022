// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

//import edu.wpi.first.wpilibj.AnalogGyro;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.drivetrain.DriveWithXboxController;
import frc.robot.subsystems.Swerve.GlobalMeasurementSensor;
import frc.robot.subsystems.Swerve.SwerveModule;
import frc.robot.subsystems.Swerve.SwervePIDConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.SPI;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI*1; // 1/2 rotation per second

  public final Translation2d m_frontLeftLocation = new Translation2d(-0.381, 0.381);
  public final Translation2d m_frontRightLocation = new Translation2d(-0.381, -0.381);
  public final Translation2d m_backLeftLocation = new Translation2d(0.381, 0.381);
  public final Translation2d m_backRightLocation = new Translation2d(0.381, -0.381);
  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
       
  //Swerve Module Motors
  private final TalonFX driveMotorM1 = new TalonFX(1);
  private final TalonFX turningMotorM1 = new TalonFX(2);
  private final TalonFX driveMotorM2 = new TalonFX(3);
  private final TalonFX turningMotorM2 = new TalonFX(4);
  private final TalonFX driveMotorM3 = new TalonFX(5);
  private final TalonFX turningMotorM3 = new TalonFX(6);
  private final TalonFX driveMotorM4 = new TalonFX(7);
  private final TalonFX turningMotorM4 = new TalonFX(8); 
  private final SwervePIDConfig swervePIDConfigM1 = new SwervePIDConfig(0.04, 0, 0.3, 0.0472, 0.1, 0.003, 0.01, 0.0475);
  private final SwervePIDConfig swervePIDConfigM2 = new SwervePIDConfig(0.04, 0, 0.3, 0.0472, 0.1, 0.003, 0.01, 0.0475);
  private final SwervePIDConfig swervePIDConfigM3 = new SwervePIDConfig(0.04, 0, 0.3, 0.0472, 0.1, 0.003, 0.01, 0.0475);
  private final SwervePIDConfig swervePIDConfigM4 = new SwervePIDConfig(0.04, 0, 0.3, 0.0472, 0.1, 0.003, 0.01, 0.0475);
  private final DutyCycleEncoder absoluteEncoderM1 = new DutyCycleEncoder(1);
  private final DutyCycleEncoder absoluteEncoderM2 = new DutyCycleEncoder(2);
  private final DutyCycleEncoder absoluteEncoderM3 = new DutyCycleEncoder(3);
  private final DutyCycleEncoder absoluteEncoderM4 = new DutyCycleEncoder(4);

  private final SwerveModule m_frontLeft = new SwerveModule(driveMotorM1, turningMotorM1, swervePIDConfigM1, absoluteEncoderM1, 0.678 , 1);
  private final SwerveModule m_frontRight = new SwerveModule(driveMotorM2, turningMotorM2, swervePIDConfigM2, absoluteEncoderM2, 0.389, 2);
  private final SwerveModule m_backLeft = new SwerveModule(driveMotorM3, turningMotorM3, swervePIDConfigM3, absoluteEncoderM3, 0.110, 3);
  private final SwerveModule m_backRight = new SwerveModule(driveMotorM4, turningMotorM4, swervePIDConfigM4, absoluteEncoderM4, 0.544, 4);

  // private final SwerveModule m_frontLeft = new SwerveModule(driveMotorM1, turningMotorM1, swervePIDConfigM1, absoluteEncoderM1, -0.280, 1);
  // private final SwerveModule m_frontRight = new SwerveModule(driveMotorM2, turningMotorM2, swervePIDConfigM2, absoluteEncoderM2, -0.153, 2);
  // private final SwerveModule m_backLeft = new SwerveModule(driveMotorM3, turningMotorM3, swervePIDConfigM3, absoluteEncoderM3, 1.287, 3);
  // private final SwerveModule m_backRight = new SwerveModule(driveMotorM4, turningMotorM4, swervePIDConfigM4, absoluteEncoderM4, 0.804, 4);

  // private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          ahrs.getRotation2d(),
          new Pose2d(),
          m_kinematics,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(Units.degreesToRadians(0.01)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public Drivetrain() {
    // m_gyro.reset();

    CommandScheduler.getInstance().setDefaultCommand(this, new DriveWithXboxController(this));
    ahrs.reset();
  }

  public void resetAHRS(){
    ahrs.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {

      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);
      m_frontLeft.setDesiredState(desiredStates[0]);
      m_frontRight.setDesiredState(desiredStates[1]);
      m_backLeft.setDesiredState(desiredStates[2]);
      m_backRight.setDesiredState(desiredStates[3]);

  }

//   public void setModuleStates(SwerveModuleState[] desiredStates) {
//     SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
//     frontLeft.setDesiredState(desiredStates[0]);
//     frontRight.setDesiredState(desiredStates[1]);
//     backLeft.setDesiredState(desiredStates[2]);
//     backRight.setDesiredState(desiredStates[3]);
// }



  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(
        ahrs.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    m_poseEstimator.addVisionMeasurement(
        GlobalMeasurementSensor.getEstimatedGlobalPose(
            m_poseEstimator.getEstimatedPosition()),
        Timer.getFPGATimestamp() - 0.3);
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(pose, getRotation2d());
  }


  public double getFrontLeftDouble() {
    return m_frontLeft.getState().speedMetersPerSecond;
  }

  public SwerveDrivePoseEstimator poseEstimator() {
    return m_poseEstimator;
  }

  public void turnWheel(double xSpeed, double ySpeed, double rot, boolean fieldRelative, int turnWheel) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    if (turnWheel == 0) {
      m_frontLeft.setDesiredState(swerveModuleStates[turnWheel]);
    }
    else if (turnWheel == 1) {
      m_frontRight.setDesiredState(swerveModuleStates[turnWheel]);
    }
    else if (turnWheel == 2) {
      m_backLeft.setDesiredState(swerveModuleStates[turnWheel]);
    }
    else {
      m_backRight.setDesiredState(swerveModuleStates[turnWheel]);
    }
  }

  public double getXSpeed() {
    return DriveWithXboxController.xSpeed();
  }
  public double getYSpeed() {
    return DriveWithXboxController.ySpeed();
  }
  public double getRot() {
    return DriveWithXboxController.rotation();
  }

  public double getHeading() {
    return Math.IEEEremainder(ahrs.getAngle(), 360);
}

  public Rotation2d getRotation2d() {
      return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {

    double x = m_poseEstimator.getEstimatedPosition().getX() * 4.1667;
    double y = m_poseEstimator.getEstimatedPosition().getY() * 4.1667;

    return new Pose2d(new Translation2d(x,y), m_poseEstimator.rotation));
  
  
  
  }

  public void stopAllModules() {
    m_frontLeft.stop();;
    m_frontRight.stop();
    m_backLeft.stop();  
    m_backRight.stop();
  }

}

