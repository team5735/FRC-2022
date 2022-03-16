package frc.robot.subsystems.shooter;

public class SpeedAngle {
    private int speed;
    private double angle;
    private double speedRatio;
    public SpeedAngle(int speed, double angle, double speedRatio) {
        this.speed = speed;
        this.angle = angle;
        this.setSpeedRatio(speedRatio);
    }
    public double getSpeedRatio() {
        return speedRatio;
    }
    public void setSpeedRatio(double speedRatio) {
        this.speedRatio = speedRatio;
    }
    public double getAngle() {
        return angle;
    }
    public void setAngle(double angle) {
        this.angle = angle;
    }
    public int getSpeed() {
        return speed;
    }
    public void setSpeed(int speed) {
        this.speed = speed;
    }
    
}
