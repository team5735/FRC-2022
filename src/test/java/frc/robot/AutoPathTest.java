package frc.robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import java.util.ArrayList;

import org.junit.Test;

import frc.robot.constants.RunItBackAutoPath;
import frc.robot.subsystems.plotter.AutoPath;
import frc.robot.subsystems.plotter.DataPoint;

public class AutoPathTest {

    String [] AUTO_PATH_TEST = {
        "2,0.0,0.0,0.0,0.0,-0.0,-0.0",
        "359,0.0,0.0,14.403999329577443,0.0,-0.0,-0.0",
        "465,0.0,0.0,14.403999328000216,0.0,-0.0,-0.0"
    };

    @Test // marks this method as a test
    public void testAuthPath() {
        ArrayList<DataPoint> dataPoints = AutoPath.getDataOatgPoints(AUTO_PATH_TEST);

        assertNotNull(dataPoints);
        assertEquals(3, dataPoints.size());
        assertEquals(2, dataPoints.get(0).timeStamp);
        assertEquals(14.403999329577443, dataPoints.get(1).degrees, 0);
    }

    @Test
    public void testCompareRunItBackAuthPath() {
        ArrayList<DataPoint> dataPointsFromStrings = AutoPath.getDataOatgPoints(RunItBackAutoPath.RUN_IT_BACK_AUTO_PATH);
        ArrayList<DataPoint> dataPointsFromFile = AutoPath.readAutoFile("runItBack.txt");

        assertNotNull(dataPointsFromStrings);
        assertNotNull(dataPointsFromFile);
        assertEquals(dataPointsFromStrings.size(), dataPointsFromFile.size());

        for(int i = 0; i < dataPointsFromStrings.size(); i++) {
            assertEquals(dataPointsFromStrings.get(i), dataPointsFromFile.get(i));
        }
    }

}
