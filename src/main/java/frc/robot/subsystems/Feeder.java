package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants;

public class Feeder {

    private CANSparkMax feederMotor;
    private final DigitalInput beambreak;

    public Feeder() {
        feederMotor = new CANSparkMax(RobotConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
        beambreak = new DigitalInput(RobotConstants.FEEDER_BEAMBREAK_DIGITAL_PORT);
    }

    public void feedShooter(double speed, boolean inverted) {
        if (inverted) {
            speed = - speed;
        }
        feederMotor.set(speed);
    }

    public boolean hasBall() {
		if (beambreak.get() == false) {
            SmartDashboard.putBoolean("beamBreak", false);
            System.out.println("################ BALL INSIDE FEEDER | BEAM BREAK");
        }

        return !beambreak.get();
	}
}
