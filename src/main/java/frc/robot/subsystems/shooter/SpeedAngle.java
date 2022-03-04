package frc.robot.subsystems.shooter;

public class SpeedAngle {
    private int speed;
    private double angle;
    public SpeedAngle(int speed, double angle) {
        this.speed = speed;
        this.angle = angle;
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
