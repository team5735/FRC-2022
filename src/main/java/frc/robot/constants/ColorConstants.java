// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/** Add your docs here. */
public class ColorConstants {

    /**
   * Change the I2C port below to match the connection of your color sensor
   */
  public static final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  public static final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  public static final ColorMatch m_colorMatcher = new ColorMatch();

  
  public static final Color kredBall = new Color(0.539, 0.343, 0.117);
  public static final Color kblueBall = new Color(0.169, 0.408, 0.425);


/**
 
  //Color Wheel colors
  public static final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  public static final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  public static final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  public static final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
 */

  public static String colorString;

}
