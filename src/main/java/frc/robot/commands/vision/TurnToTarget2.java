// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTarget2 extends PIDCommand {
  Vision visionSubsystem;
  public TurnToTarget2(Vision visionSubsystem, Drivetrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(1, 0.0001, 0.1),
        // This should return the measurement
        visionSubsystem::getTX,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          drivetrain.drive(0, 0, output, false);
        });
        getController()
        .setTolerance(DriveConstants.TURN_TOLERANCE_DEG, DriveConstants.TURN_RATE_TOLERANCE_DEG_PER_SECOND);
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
