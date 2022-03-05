package frc.robot.subsystems.shooter;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.LoggingConstants;
import frc.robot.constants.LoggingConstants.LoggingLevel;

public class TargetMapper {
    public static final Map<Integer, SpeedAngle> targetMapping = Map.ofEntries(
        //Key: distance by foot
        Map.entry(3,new SpeedAngle(9000, 0)),
        Map.entry(4,new SpeedAngle(9100, 0.05)),
        Map.entry(5,new SpeedAngle(9200, 0.1)),
        Map.entry(6,new SpeedAngle(9300, 0.15)),
        Map.entry(7,new SpeedAngle(9400, 0.2)),
        Map.entry(8,new SpeedAngle(9500, 0.25)),
        Map.entry(9,new SpeedAngle(9600, 0.3)),
        Map.entry(10,new SpeedAngle(9700, 0.35)),
        Map.entry(11,new SpeedAngle(9800, 0.4)),
        Map.entry(12,new SpeedAngle(9900, 0.45)),
        Map.entry(13,new SpeedAngle(10000, 0.45)),
        Map.entry(14,new SpeedAngle(10100, 0.45)),
        Map.entry(15,new SpeedAngle(10200, 0.5)),
        Map.entry(16,new SpeedAngle(10300, 0.55)),
        Map.entry(17,new SpeedAngle(10400, 0.6)),
        Map.entry(18,new SpeedAngle(10500, 0.6)),
        Map.entry(19,new SpeedAngle(11000, 0.6)),
        Map.entry(20,new SpeedAngle(11000, 0.68)),
        Map.entry(21,new SpeedAngle(11900, 0.7)),
        Map.entry(22,new SpeedAngle(12000, 0.75)),//ok
        Map.entry(23,new SpeedAngle(12000, 0.75)),
        Map.entry(24,new SpeedAngle(14000, 0.75))
    );

    public static SpeedAngle getSpeedAngleByDistance(double distance){
        if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
            SmartDashboard.putNumber("Input Distance", distance);
          }

        if (distance< 36)
            return targetMapping.get(3);
        if (distance > 288)
            return targetMapping.get(24);
        SpeedAngle speedAngle = targetMapping.get((int)Math.round(distance/12));
        if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
            SmartDashboard.putNumber("round Feet", (int)Math.round(distance/12));
            SmartDashboard.putNumber("Output Speed", speedAngle.getSpeed());
            SmartDashboard.putNumber("Output Angle", speedAngle.getAngle());
          }
        return speedAngle;
    }
}
