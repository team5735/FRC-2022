package frc.robot.constants;

public class RobotConstants {
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int SUBSYSTEM_CONTROLLER_PORT = 1;

    public static final int INTAKE_MOTOR_ID = 9;
    public static final int BIG_SHOOTER_MASTER_MOTOR_ID = 56;
    public static final int BIG_SHOOTER_FOLLOWER_MOTOR_ID = 58;
    public static final int SMALL_SHOOTER_MOTOR_ID = 57;
    public static final int FEEDER_MOTOR_ID = 10;
    public static final int HOOD_MOTOR_ID = 11;

    public static final int HOOD_MOTOR_CURRENT_LIMIT = 2;
    public static final int HOOD_ENCODER_DIO_PORT = 5;

    // digital input ports
    public static final int FEEDER_BEAMBREAK_DIGITAL_PORT = 6;

    //Game Field Values
    // public static final double TARGET_HEIGHTFROMGROUND = Units.inchesToMeters(/*Test Val*/ 31.5); // Original: 104 in //Returns Meters
    public static final double TARGET_HEIGHTFROMGROUND = /*Test Val*/ 104;

    //Camera Values
    // public static final double CAMERA_HEIGHTFROMGROUND = Units.inchesToMeters(/*Test Val*/ 25); //Original: 27 in // Returns Meters
    public static final double CAMERA_HEIGHTFROMGROUND = /*Test Val*/ 32.5;

    // public static final double CAMERA_ANGLEFROMPARALLEL = Units.degreesToRadians(/*Test Val*/ 17); // Original: 50 deg //Returns Rads
    public static final double CAMERA_ANGLEFROMPARALLEL = /*Test Val*/ 36; // Original: 50 deg //Returns Rads

}