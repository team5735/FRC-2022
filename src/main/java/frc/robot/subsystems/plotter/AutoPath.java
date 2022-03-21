package frc.robot.subsystems.plotter;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;

public class AutoPath {

    public static ArrayList<DataPoint> getDataOatgPoints(String [] autoPath) {
        ArrayList<DataPoint> dataPathPoints = new ArrayList<>();

        if (autoPath != null && autoPath.length > 0) {
            for (String data : autoPath) {
                String[] dataPointStr = data.split(",");

                long timeStamp = Long.parseLong(dataPointStr[0]);
                double xPos = Double.parseDouble(dataPointStr[1]);
                double yPos = Double.parseDouble(dataPointStr[2]);
                double angle = Double.parseDouble(dataPointStr[3]);
                double actRot = Double.parseDouble(dataPointStr[4]);
                double xS = Double.parseDouble(dataPointStr[5]);
                double yS = Double.parseDouble(dataPointStr[6]);

                DataPoint dataPoint = new DataPoint(timeStamp, xPos, yPos, angle, actRot, xS, yS);
                dataPathPoints.add(dataPoint);
            }
        }

        return dataPathPoints;
    }

    public static ArrayList<DataPoint> readAutoFile(String filename) {

        ArrayList<DataPoint> dataPathPoints = new ArrayList<>();
    
        try {
            File file = new File(Filesystem.getDeployDirectory().toPath().resolve(filename).toString());
            Scanner myReader = new Scanner(file);
    
            while (myReader.hasNextLine()) {
                String data = myReader.nextLine();
                //List<String> dataPoint = Arrays.asList(data.split(","));
                String[] dataPointStr = data.split(",");
                long timeStamp = Long.parseLong(dataPointStr[0]);
                double xPos = Double.parseDouble(dataPointStr[1]);
                double yPos = Double.parseDouble(dataPointStr[2]);
                double angle = Double.parseDouble(dataPointStr[3]);
                double actRot = Double.parseDouble(dataPointStr[4]);
                double xS = Double.parseDouble(dataPointStr[5]);
                double yS = Double.parseDouble(dataPointStr[6]);
    
                DataPoint dataPoint = new DataPoint(timeStamp, xPos, yPos, angle, actRot, xS, yS);
                dataPathPoints.add(dataPoint);
            }
    
            myReader.close();
        } catch (FileNotFoundException e) {
            //System.out.println("An error occurred.");
            e.printStackTrace();
        }
    
        return dataPathPoints;
    }
    
}
