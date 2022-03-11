package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterWheelsSubsystem;

public class ShooterWheelsReverse extends CommandBase {
    private ShooterWheelsSubsystem subsystem;
    public ShooterWheelsReverse(ShooterWheelsSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        subsystem.reverse();
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
    
}
