package frc.robot.commands.shooter;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.junit.Test;

import frc.robot.subsystems.shooter.FeederSubsystem;

public class FeederReverseForShootCommandTest {

    @Before
    public void setup() {
    }

    @Test
    public void testFeederReverseForShootHasBall() throws InterruptedException {
        FeederSubsystem feeder = mock(FeederSubsystem.class);

        FeederReverseForShoot command = new FeederReverseForShoot(feeder);
        when(feeder.hasBall()).thenReturn(true);

        command.initialize();
        verify(feeder).feederReverseForShoot();
        assertEquals(command.isFinished(), false);
    }

    @Test
    public void testFeederReverseForShootNotHasBall() throws InterruptedException {
        FeederSubsystem feeder = mock(FeederSubsystem.class);

        FeederReverseForShoot command = new FeederReverseForShoot(feeder);
        when(feeder.hasBall()).thenReturn(false);

        command.initialize();
        verify(feeder, times(0)).feederReverseForShoot();
        assertEquals(command.isFinished(), true);
    }

}
