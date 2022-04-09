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
        Map.entry(8,new SpeedAngle(9004, 0.2, 1.5)),
        Map.entry(9,new SpeedAngle(9400, 0.28, 1.5)),
        Map.entry(10,new SpeedAngle(9769, 0.37, 1.5)),
        Map.entry(11,new SpeedAngle(10116, 0.45, 1.5)),
        Map.entry(12,new SpeedAngle(10443, 0.53, 1.5)),
        Map.entry(13,new SpeedAngle(10753, 0.61, 1.5)),
        Map.entry(14,new SpeedAngle(11048, 0.70, 1.5)),
        Map.entry(15,new SpeedAngle(11331, 0.78, 1.5)),
        Map.entry(16,new SpeedAngle(11601, 0.86, 1.5)),
        Map.entry(17,new SpeedAngle(11861, 0.93, 1.5)),
        Map.entry(18,new SpeedAngle(12112, 0.93, 1.5)),
        Map.entry(19,new SpeedAngle(12500, 0.93, 1.5)),
        Map.entry(20,new SpeedAngle(12916, 0.93, 1.5)),
        Map.entry(21,new SpeedAngle(13333, 0.93, 1.5)),
        Map.entry(22,new SpeedAngle(13750, 0.93, 1.5)),//ok
        Map.entry(23,new SpeedAngle(14166, 0.93, 1.5)),
        Map.entry(24,new SpeedAngle(14583, 0.93, 1.5)),
        Map.entry(25,new SpeedAngle(15000, 0.93, 1.5)),
        Map.entry(26,new SpeedAngle(15416, 0.93, 1.5))
    );

    public static SpeedAngle getSpeedAngleByDistance(double distance){

        if (distance < 90)
            return targetMapping.get(7);
        if (distance > 216) {
            double speed = 34.722 * distance + 4583;
            SpeedAngle speedAngle = new SpeedAngle((int)speed, 0.93, 1.5);
            return speedAngle;
            //return targetMapping.get(24);
        }

        double speed = 1696 * Math.pow(distance, 0.3657);

        SpeedAngle floorSpeedAngle = targetMapping.get((int)Math.floor(distance/12));
        SpeedAngle ceilSpeedAngle = targetMapping.get((int)Math.floor((distance+12)/12));

        double ratio = (distance - Math.floor(distance/12) * 12) / 12;

        SpeedAngle speedAngle = new SpeedAngle(
            (int) Math.max(speed, 10000),
            floorSpeedAngle.getAngle() + (ceilSpeedAngle.getAngle() - floorSpeedAngle.getAngle()) * ratio,
            distance < 90? 1:1.5
        );
        
        if (LoggingConstants.SHOOTER_LEVEL.ordinal() >= LoggingLevel.DEBUG.ordinal()) {
            SmartDashboard.putNumber("Input Distance", distance);
            SmartDashboard.putNumber("Output Func", speed);
            SmartDashboard.putNumber("Output Speed", speedAngle.getSpeed());
            SmartDashboard.putNumber("Output Angle", speedAngle.getAngle());
        }

        return speedAngle;
    }
}
