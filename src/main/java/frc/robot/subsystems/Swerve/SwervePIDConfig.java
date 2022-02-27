package frc.robot.subsystems.Swerve;

public class SwervePIDConfig{
    private PIDConfig driveMotorPID;
    private PIDConfig turningMotorPID;
    
    public SwervePIDConfig(double driveKp, double driveKi, double driveKd, double driveKf, double turningKp, double turningKi, double turningKd, double turningKf){
        driveMotorPID = new PIDConfig(driveKp, driveKi, driveKd, driveKf);
        turningMotorPID = new PIDConfig(turningKp, turningKi, turningKd, turningKf);
    }

    public PIDConfig getDriveMotorPID() {
        return driveMotorPID;
    }

    public PIDConfig getTurningMotorPID() {
        return turningMotorPID;
    }

    static public class PIDConfig {
        private double kp, ki, kd, kf;
        public PIDConfig(double kp, double ki, double kd, double kf) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.kf = kf;
        }

        public double getKp() {
            return kp;
        }

        public double getKi() {
            return ki;
        }

        public double getKd() {
            return kd;
        }

        public double getKf() {
            return kf;
        }
    }
}


