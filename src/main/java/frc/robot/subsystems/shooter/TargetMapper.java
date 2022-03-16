package frc.robot.subsystems.shooter;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class TargetMapper {
    public static final Map<Integer, SpeedAngle> targetMapping = Map.ofEntries(
        //Key: distance by foot
        Map.entry(3,new SpeedAngle(9000, 0.06, 1.0)),
        Map.entry(4,new SpeedAngle(9100, 0.06, 1.0)),
        Map.entry(5,new SpeedAngle(9200, 0.06, 1.0)),
        Map.entry(6,new SpeedAngle(9300, 0.06, 1.0)),
        Map.entry(7,new SpeedAngle(9400, 0.06, 1.0)),
        Map.entry(8,new SpeedAngle(9500, 0.26, 1.2)),
        Map.entry(9,new SpeedAngle(9600, 0.29, 1.5)),
        Map.entry(10,new SpeedAngle(9700, 0.32, 1.5)),
        Map.entry(11,new SpeedAngle(10200, 0.36, 1.5)),
        Map.entry(12,new SpeedAngle(9900, 0.39, 1.5)),
        Map.entry(13,new SpeedAngle(10000, 0.42, 1.5)),
        Map.entry(14,new SpeedAngle(10100, 0.45, 1.5)),
        Map.entry(15,new SpeedAngle(10200, 0.49, 1.5)),
        Map.entry(16,new SpeedAngle(10300, 0.52, 1.5)),
        Map.entry(17,new SpeedAngle(10400, 0.55, 1.5)),
        Map.entry(18,new SpeedAngle(10500, 0.58, 1.5)),
        Map.entry(19,new SpeedAngle(11000, 0.61, 1.5)),
        Map.entry(20,new SpeedAngle(11000, 0.65, 1.5)),
        Map.entry(21,new SpeedAngle(11900, 0.68, 1.5)),
        Map.entry(22,new SpeedAngle(12000, 0.71, 1.5)),//ok
        Map.entry(23,new SpeedAngle(12000, 0.74, 1.5)),
        Map.entry(24,new SpeedAngle(14000, 0.75, 1.5))
    );

    public static SpeedAngle getSpeedAngleByDistance(double distance){
        if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
            SmartDashboard.putNumber("Input Distance", distance);
          }

        if (distance < 90)
            return targetMapping.get(7);
        if (distance > 288)
            return targetMapping.get(24);

        double speed = 3595 * Math.pow(distance, 0.2494);

        SpeedAngle floorSpeedAngle = targetMapping.get((int)Math.floor(distance/12));
        SpeedAngle ceilSpeedAngle = targetMapping.get((int)Math.floor((distance+12)/12));

        double ratio = (distance - Math.floor(distance/12) * 12) / 12;

        SpeedAngle speedAngle = new SpeedAngle(
            (int) Math.min(Math.max(speed, 8500), 10000),
            floorSpeedAngle.getAngle() + (ceilSpeedAngle.getAngle() - floorSpeedAngle.getAngle()) * ratio,
            distance < 90? 1:1.5
        );

        return speedAngle;
    }
}
