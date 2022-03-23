// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import frc.robot.RobotContainer;
import frc.robot.constants.ColorConstants;

public class ColorSensor {
    
    private final I2C.Port i2cPort;
    private final ColorSensorV3 m_colorSensor;
    private final ColorMatch m_colorMatcher;
    private final String teamColor;
    private String colorString;
    private String currentColor;

    public ColorSensor() {

        /**
        * Change the I2C port below to match the connection of color sensor
        */
        i2cPort = I2C.Port.kOnboard;

        /**
         * A Rev Color Sensor V3 object is constructed with an I2C port as a 
         * parameter. The device will be automatically initialized with default 
         * parameters.
         */
        m_colorSensor = new ColorSensorV3(i2cPort);
    
        /**
         * A Rev Color Match object is used to register and detect known colors. This can 
         * be calibrated ahead of time or during operation.
         * 
         * This object uses a simple euclidian distance to estimate the closest match
         * with given confidence range.
         */
        m_colorMatcher = new ColorMatch();
    
        m_colorMatcher.addColorMatch(ColorConstants.K_BLUE_BALL);
        //ColorConstants.m_colorMatcher.addColorMatch(ColorConstants.kGreenTarget);
        m_colorMatcher.addColorMatch(ColorConstants.K_RED_BALL);
        //ColorConstants.m_colorMatcher.addColorMatch(ColorConstants.kYellowTarget); 
        m_colorMatcher.addColorMatch(ColorConstants.NO_BALL);
    
        if (ColorConstants.ALLIANCE_COLOR.toString().equals("Red")) {
            teamColor = "red";
        } else {
            teamColor = "blue";
        }
    
        colorString = teamColor;

        currentColor = RobotContainer.colorChooser.getSelected();
    }

    public void senseColor(double intakeMotorSpeed) {
        
        /**
         * The method GetColor() returns a normalized color value from the sensor and can be
         * useful if outputting the color to an RGB LED or similar. To
         * read the raw color, use GetRawColor().
         * 
         * The color sensor works best when within a few inches from an object in
         * well lit conditions (the built in LED is a big help here!). The farther
         * an object is the more light from the surroundings will bleed into the 
         * measurements and make it difficult to accurately determine its color.
         */
        Color detectedColor = m_colorSensor.getColor();

        /**
         * Run the color match algorithm on our detected color
         */
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == ColorConstants.K_BLUE_BALL) {
            //set led to blue
            colorString = "Blue";
        } else if (match.color == ColorConstants.K_RED_BALL) {
            // set led to red
            colorString = "Red";
        } //if (match.color == ColorConstants.NO_BALL) {//maintain color = don't change anything}

        // If intake running out, set color to teamColor (default)
        if (intakeMotorSpeed < 0) {
            colorString = teamColor;
        }

        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);
        SmartDashboard.putString("Alliance", ColorConstants.ALLIANCE_COLOR.toString());

        SmartDashboard.putBoolean("Ball Color", colorString.equals(currentColor));
    }

}
