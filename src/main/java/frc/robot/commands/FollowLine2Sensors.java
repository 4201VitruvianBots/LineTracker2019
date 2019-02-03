/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Command to follow a line with 2 sensors
 */
public class FollowLine2Sensors extends Command {
    public final double SPEED_MODIFIER = 0.001;
    public final double DELTA_VALUE = 500; // Difference between readings over dark and light surfaces
    public double initRightSensorValue = Robot.lineTracker.getRightSensorValue(); // Value when right sensor is over carpet
    public double initLeftSensorValue = Robot.lineTracker.getLeftSensorValue(); // Value when left sensor is over carpet
    public double targetRightValue = initRightSensorValue - DELTA_VALUE;    // Target value for the right sensor
    public double targetLeftValue = initLeftSensorValue - DELTA_VALUE;  // Target value for the left sensor

    public FollowLine2Sensors() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.lineTracker);
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        // Get sensor values
        double leftSensorValue = Robot.lineTracker.getLeftSensorValue();
        double rightSensorValue = Robot.lineTracker.getRightSensorValue();

        // Calculate speed
        double leftSpeed = leftSensorValue * SPEED_MODIFIER;
        double rightSpeed = rightSensorValue * SPEED_MODIFIER;

        Robot.driveTrain.setMotorPercentOutput(leftSpeed, rightSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if (Robot.lineTracker.getRightSensorValue() <= targetRightValue && Robot.lineTracker.getLeftSensorValue() <= targetLeftValue) // Both sensors see the tape
        {
            return true;
        } else
        {
            return false;
        }
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.setMotorPercentOutput(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}