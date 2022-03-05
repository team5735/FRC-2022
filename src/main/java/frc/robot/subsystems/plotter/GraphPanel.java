package frc.robot.subsystems.plotter;

import javax.swing.JPanel;
import java.util.Iterator;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;

public class GraphPanel extends JPanel {

    private ArrayList<DataPoint> dataPoints = null;
    Graphics2D g2 = null;
    private int radius;

    public void setDataPoints(ArrayList<DataPoint> dataPoints)
    {
        this.dataPoints = dataPoints;
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        g2 = (Graphics2D) g;
        //could be done by declaring panel as a constructor
        draw(g2);
    }
      public void draw(Graphics2D g2){

        int panelHeight = getHeight();
        int panelWidth = getWidth();
        int xAxisLocation = panelWidth / 2;
        int yAxisLocation = panelHeight / 2;

        /* Draw Charting Lines
           May eventually need to be converted to inches
           Tiles on floor and tape measures are all in inches
        */
        g2.setColor(new Color(173,216,230));
        for (int i = 0; i < panelHeight / 50; i++) {
            g2.drawLine(0, yAxisLocation+50*i, panelWidth, yAxisLocation+50*i);
            g2.drawLine(0, yAxisLocation-50*i, panelWidth, yAxisLocation-50*i);
        }
        for (int i = 0; i < panelWidth / 50; i++) {
            g2.drawLine(xAxisLocation+50*i,0,xAxisLocation+50*i, panelHeight); 
            g2.drawLine(xAxisLocation-50*i,0,xAxisLocation-50*i, panelHeight);
        }
        // Draw axis
        g2.setColor(Color.BLACK);
        g2.drawLine(xAxisLocation,0,xAxisLocation, panelHeight);
        g2.drawLine(0, yAxisLocation, panelWidth, yAxisLocation);
        g2.drawString("One Section is 50 cm", 0, panelHeight-5);
        
        Iterator<DataPoint> iter = dataPoints.iterator();
        int counter = 0;
        while(iter.hasNext())
        {
            
            DataPoint dataPoint = iter.next();
            int xPos = (int)(dataPoint.x * 200 + xAxisLocation);
            int yPos = (int)(dataPoint.y * -200 + yAxisLocation);
            double radians = Math.toRadians(dataPoint.degrees);

            if(counter % 10 == 0) {
                g2.drawOval(xPos-radius, yPos-radius, radius*2, radius*2);
                g2.drawLine(xPos,yPos,(int)(xPos+radius*5*Math.cos(radians)),(int)(yPos-radius*5*Math.sin(radians)));
            }
            g2.setColor(Color.green);
            //g2.drawOval(xPos-1, yPos-1, 1, 1);
            g2.setColor(Color.black);
            
            counter ++;
        }
      }

      public void setRadius(int a) {
          radius = a;
      }
}
