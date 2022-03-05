// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import frc.robot.constants.ColorConstants;

public class ColorSensor extends SubsystemBase {

  /** Creates a new ColorSensor. */
  public ColorSensor() {

    ColorConstants.m_colorMatcher.addColorMatch(ColorConstants.kblueBall);
    //ColorConstants.m_colorMatcher.addColorMatch(ColorConstants.kGreenTarget);
    ColorConstants.m_colorMatcher.addColorMatch(ColorConstants.kredBall);
    //ColorConstants.m_colorMatcher.addColorMatch(ColorConstants.kYellowTarget); 

    System.out.println("color sensor created");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    System.out.println("color sensor running");
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
    Color detectedColor = ColorConstants.m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = ColorConstants.m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == ColorConstants.kblueBall) {
      colorString = "Blue";
    } else if (match.color == ColorConstants.kredBall) {
      colorString = "Red";
    } 
    
    /**
     //Color Wheel
     else if (match.color == ColorConstants.kGreenTarget) {
      colorString = "Green";
    } else if (match.color == ColorConstants.kYellowTarget) {
      colorString = "Yellow";
    } 
    */
    else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

  }
}
