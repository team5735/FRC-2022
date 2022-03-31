package frc.robot.commands.shooter;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.junit.Test;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterWheelsSubsystem;

public class FeederForwardCommandTest {

    @Before
    public void setup() {
    }

    @Test
    public void testFeeder() throws InterruptedException {
        FeederSubsystem feeder = mock(FeederSubsystem.class);

        FeederForward command = new FeederForward(feeder);
        command.initialize();

        verify(feeder).feederForward();
    }

    @Test
    public void testConditioalFeeder() {
        FeederSubsystem feeder = mock(FeederSubsystem.class);
        ShooterWheelsSubsystem shooter = mock(ShooterWheelsSubsystem.class);
        when(shooter.atSpeed()).thenReturn(true);

        ConditionalCommand command = new ConditionalCommand(new FeederForward(feeder),
            new FeederStop(feeder), shooter::atSpeed);

        command.initialize();
        command.execute();

        verify(feeder).feederForward();
    }
}
