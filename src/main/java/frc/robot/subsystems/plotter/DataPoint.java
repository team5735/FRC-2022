package frc.robot.subsystems.plotter;

public class DataPoint {
    public final long timeStamp;
    public final double x, y, degrees, actualRot, xSpeed, ySpeed;

    public DataPoint(long timeStamp, double x, double y, double degrees,
        double actualRot, double xSpeed, double ySpeed) {
        this.timeStamp=timeStamp;
        this.x=x;
        this.y=y;
        this.degrees=degrees;
        this.actualRot = actualRot;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
    }

}
