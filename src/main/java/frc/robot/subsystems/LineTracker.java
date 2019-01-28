/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Subsystem for the line tracker. Reads values from the an Arduino connected to the line tracker sensor
 */
public class LineTracker extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DigitalInput[] digitalInputs = {
          new DigitalInput(RobotMap.DIO_1), // Left side sensor
          new DigitalInput(RobotMap.DIO_2), // Middle sensor
          new DigitalInput(RobotMap.DIO_3)  // Right side sensor
  };

  public Boolean getLeftPinStatus() {
    return digitalInputs[0].get();
  }

  public Boolean getMiddlePinStatus() {
    return digitalInputs[1].get();
  }

  public Boolean getRightPinStatus() {
    return digitalInputs[2].get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}