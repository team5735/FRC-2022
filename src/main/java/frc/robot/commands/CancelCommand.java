package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CancelCommand extends CommandBase {

    private Command commandToCancel;

    public CancelCommand(Command commandToCancel) {
        this.commandToCancel = commandToCancel;
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        if (commandToCancel != null)
            commandToCancel.cancel();;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
}
