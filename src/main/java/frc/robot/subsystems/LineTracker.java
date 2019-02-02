/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Subsystem for the line tracker. Reads values from the an Arduino connected to the line tracker sensor
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

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}