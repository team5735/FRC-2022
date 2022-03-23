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

    @Override
    public boolean equals(Object obj) {
        if (!(obj  == null || obj instanceof DataPoint)) {
            return false;
        }

        DataPoint second = (DataPoint) obj;
        if (this.timeStamp == second.timeStamp &&
            this.x == second.x && this.y == second.y &&
            this.degrees == second.degrees && this.actualRot == second.actualRot &&
            this.xSpeed == second.xSpeed && this.ySpeed == second.ySpeed) {
            return true;
        } else {
            return false;
        }
    }

}
