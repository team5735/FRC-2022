package frc.robot.subsystems.shooter;

import java.util.Map;

public class TargetMapper {
    public static final Map<Integer, SpeedAngle> targetMapping = Map.ofEntries(
        Map.entry(0,new SpeedAngle(5000, 0)),
        Map.entry(40,new SpeedAngle(5000, 0)),
        Map.entry(50,new SpeedAngle(5000, 0)),
        Map.entry(60,new SpeedAngle(5000, 5)),
        Map.entry(70,new SpeedAngle(5000, 5)),
        Map.entry(80,new SpeedAngle(5000, 10)),
        Map.entry(90,new SpeedAngle(7500, 10)),
        Map.entry(100,new SpeedAngle(7500, 15)),
        Map.entry(110,new SpeedAngle(7500, 15)),
        Map.entry(120,new SpeedAngle(7500, 20)),
        Map.entry(130,new SpeedAngle(7500, 20)),
        Map.entry(140,new SpeedAngle(7500, 25)),
        Map.entry(150,new SpeedAngle(10000, 25)),
        Map.entry(160,new SpeedAngle(10000, 30)),
        Map.entry(170,new SpeedAngle(10000, 30)),
        Map.entry(180,new SpeedAngle(10000, 30)),
        Map.entry(190,new SpeedAngle(10000, 40)),
        Map.entry(200,new SpeedAngle(10000, 45)),
        Map.entry(210,new SpeedAngle(10000, 50)),
        Map.entry(220,new SpeedAngle(10000, 55)),
        Map.entry(230,new SpeedAngle(10000, 60)),
        Map.entry(240,new SpeedAngle(15000, 65)),
        Map.entry(250,new SpeedAngle(15000, 70)),
        Map.entry(260,new SpeedAngle(15000, 75)),
        Map.entry(270,new SpeedAngle(15000, 75)),
        Map.entry(280,new SpeedAngle(15000, 75))
    );

    public static SpeedAngle getSpeedAngleByDistance(double distance){
        if (distance< 40)
            return targetMapping.get(0);
        if (distance > 280)
            return targetMapping.get(280);
        System.out.println("###########" + Math.round(distance/10)*10);
        return targetMapping.get((int)(Math.round(distance/10)*10));
    }
}
