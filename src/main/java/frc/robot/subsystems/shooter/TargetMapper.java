package frc.robot.subsystems.shooter;

import java.util.Map;

public class TargetMapper {
    public static final Map<Integer, SpeedAngle> targetMapping = Map.ofEntries(
        Map.entry(3,new SpeedAngle(7500, 0)),
        Map.entry(4,new SpeedAngle(7600, 0.05)),
        Map.entry(5,new SpeedAngle(7700, 0.1)),
        Map.entry(6,new SpeedAngle(7800, 0.15)),
        Map.entry(7,new SpeedAngle(7900, 0.2)),
        Map.entry(8,new SpeedAngle(8000, 0.25)),
        Map.entry(9,new SpeedAngle(8100, 0.3)),
        Map.entry(10,new SpeedAngle(8200, 0.35)),
        Map.entry(11,new SpeedAngle(8300, 0.4)),
        Map.entry(12,new SpeedAngle(8400, 0.45)),
        Map.entry(13,new SpeedAngle(9000, 0.45)),
        Map.entry(14,new SpeedAngle(9500, 0.45)),
        Map.entry(15,new SpeedAngle(10000, 0.5)),
        Map.entry(16,new SpeedAngle(10500, 0.55)),
        Map.entry(17,new SpeedAngle(11000, 0.6)),
        Map.entry(18,new SpeedAngle(12000, 0.6)),
        Map.entry(19,new SpeedAngle(13000, 0.6)),
        Map.entry(20,new SpeedAngle(13500, 0.6)),
        Map.entry(21,new SpeedAngle(14000, 0.6)),
        Map.entry(22,new SpeedAngle(14500, 0.70)),
        Map.entry(23,new SpeedAngle(15000, 0.75)),
        Map.entry(24,new SpeedAngle(15500, 0.75))
    );

    public static SpeedAngle getSpeedAngleByDistance(double distance){
        if (distance< 36)
            return targetMapping.get(3);
        if (distance > 288)
            return targetMapping.get(24);
        return targetMapping.get((int)Math.round(distance/12));
    }
}
