/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Subsystem for the line tracker. Reads values from line tracker sensors connected to RoboRIO analog ports
 */
public class LineTracker extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public AnalogInput[] analogInputs = {
          new AnalogInput(0), // Middle sensor
          new AnalogInput(1), // Right sensor
          new AnalogInput(2) // Left sensor
  };

  public double getLeftSensorValue() {
    return analogInputs[2].getValue();
  }

  public double getCenterSensorValue() {
    return analogInputs[0].getValue();
  }

  public double getRightSensorValue() {
    return analogInputs[1].getValue();
  }

  public void updateSmartDashboard(){
    SmartDashboard.putNumber("Left Sensor Value", getLeftSensorValue());
    SmartDashboard.putNumber("Right Sensor Value", getRightSensorValue());
    SmartDashboard.putNumber("Center Sensor Value", getCenterSensorValue());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}