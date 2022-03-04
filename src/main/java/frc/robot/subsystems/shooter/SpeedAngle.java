package frc.robot.subsystems.shooter;

public class SpeedAngle {
    private int speed;
    private int angle;
    public SpeedAngle(int speed, int angle) {
        this.speed = speed;
        this.angle = angle;
    }
    public int getAngle() {
        return angle;
    }
    public void setAngle(int angle) {
        this.angle = angle;
    }
    public int getSpeed() {
        return speed;
    }
    public void setSpeed(int speed) {
        this.speed = speed;
    }
}
