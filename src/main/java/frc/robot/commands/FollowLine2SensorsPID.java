/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.vitruvianlib.util.DummyPIDInput;
import frc.vitruvianlib.util.DummyPIDOutput;

/**
 * Command to follow a line with 2 sensors
 */
public class FollowLine2SensorsPID extends Command {
    public final double SPEED_MODIFIER = 0.001;
    public final double DELTA_VALUE = 500; // Difference between readings over dark and light surfaces
    public double initRightSensorValue;
    public double initLeftSensorValue;
    public double targetRightValue;
    public double targetLeftValue;

    public double kP = 0.03;
    public double kI = 0;
    public double kD = 0;
    DummyPIDInput leftInput, rightInput;
    DummyPIDOutput leftOutput = new DummyPIDOutput();
    DummyPIDOutput rightOutput = new DummyPIDOutput();
    PIDController leftPID, rightPID;

    public FollowLine2SensorsPID() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.lineTracker);
        requires(Robot.driveTrain);

        initRightSensorValue = Robot.lineTracker.getRightSensorValue(); // Value when right sensor is over carpet
        initLeftSensorValue = Robot.lineTracker.getLeftSensorValue(); // Value when left sensor is over carpet

        targetRightValue = initRightSensorValue - DELTA_VALUE;    // Target value for the right sensor
        targetLeftValue = initLeftSensorValue - DELTA_VALUE;  // Target value for the left sensor

        leftPID = new PIDController(kP, kI, kD, Robot.lineTracker.analogInputs[0], leftOutput);
        leftPID.setOutputRange(-0.2, 0.2);

        rightPID = new PIDController(kP, kI, kD, Robot.lineTracker.analogInputs[2], rightOutput);
        rightPID.setOutputRange(-0.2, 0.2);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        leftPID.setSetpoint(targetRightValue);

        rightPID.setSetpoint(targetRightValue);

        leftPID.enable();
        rightPID.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.driveTrain.setMotorPercentOutput(leftOutput.getOutput(), rightOutput.getOutput());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return leftPID.onTarget() && rightPID.onTarget();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        leftPID.disable();
        rightPID.disable();
        Robot.driveTrain.setMotorPercentOutput(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}