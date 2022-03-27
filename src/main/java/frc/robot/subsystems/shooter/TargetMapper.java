package frc.robot.subsystems.shooter;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class TargetMapper {
    public static final Map<Integer, SpeedAngle> targetMapping = Map.ofEntries(
        //Key: distance by foot
        Map.entry(3,new SpeedAngle(10000, 0.06, 1.5)),
        Map.entry(4,new SpeedAngle(10000, 0.06, 1.5)),
        Map.entry(5,new SpeedAngle(10000, 0.06, 1.5)),
        Map.entry(6,new SpeedAngle(10000, 0.06, 1.5)),
        Map.entry(7,new SpeedAngle(10000, 0.06, 1.5)),
        Map.entry(8,new SpeedAngle(8728, 0.18, 1.5)),
        Map.entry(9,new SpeedAngle(9096, 0.23, 1.5)),
        Map.entry(10,new SpeedAngle(9438, 0.29, 1.5)),
        Map.entry(11,new SpeedAngle(9758, 0.34, 1.5)),
        Map.entry(12,new SpeedAngle(10060, 0.40, 1.5)),
        Map.entry(13,new SpeedAngle(10346, 0.45, 1.5)),
        Map.entry(14,new SpeedAngle(10618, 0.51, 1.5)),
        Map.entry(15,new SpeedAngle(10878, 0.57, 1.5)),
        Map.entry(16,new SpeedAngle(11126, 0.62, 1.5)),
        Map.entry(17,new SpeedAngle(11365, 0.68, 1.5)),
        Map.entry(18,new SpeedAngle(11595, 0.73, 1.5)),
        Map.entry(19,new SpeedAngle(11817, 0.79, 1.5)),
        Map.entry(20,new SpeedAngle(12031, 0.84, 1.5)),
        Map.entry(21,new SpeedAngle(12238, 0.90, 1.5)),
        Map.entry(22,new SpeedAngle(12439, 0.95, 1.5)),//ok
        Map.entry(23,new SpeedAngle(12634, 0.95, 1.5)),
        Map.entry(24,new SpeedAngle(13500, 0.95, 1.5))
    );

    public static SpeedAngle getSpeedAngleByDistance(double distance){
        if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
            SmartDashboard.putNumber("Input Distance", distance);
          }

        if (distance < 90)
            return targetMapping.get(7);
        if (distance > 288)
            return targetMapping.get(24);

        double speed = 1765 * Math.pow(distance, 0.3502);

        SpeedAngle floorSpeedAngle = targetMapping.get((int)Math.floor(distance/12));
        SpeedAngle ceilSpeedAngle = targetMapping.get((int)Math.floor((distance+12)/12));

        double ratio = (distance - Math.floor(distance/12) * 12) / 12;

        SpeedAngle speedAngle = new SpeedAngle(
            (int) Math.max(speed, 10000),
            floorSpeedAngle.getAngle() + (ceilSpeedAngle.getAngle() - floorSpeedAngle.getAngle()) * ratio,
            distance < 90? 1:1.5
        );

        SmartDashboard.putNumber("$Output Func", speed);
        SmartDashboard.putNumber("Output Speed", speedAngle.getSpeed());
        SmartDashboard.putNumber("Output Angle", speedAngle.getAngle());

        return speedAngle;
    }
}
