/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class HoldToAlignWithTarget extends PIDCommand {
    static double kP = 0.04;   //0.1
    static double kI = 0;
    static double kD = 0;  //10
    static double kF = 0;  //1023.0 / 72000.0;
    static double period = 0.02;
    double lastLimelightAngle = 0;
    double turn;
    //Notifier periodicRunnable;
    boolean isFinished = false;

    public HoldToAlignWithTarget() {
        super(kP, kI, kD, period);
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
        requires(Robot.vision);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.driveTrain.setDriveMotorsState(false);
        getPIDController().setF(kF);
        getPIDController().setContinuous(true);
        getPIDController().setAbsoluteTolerance(1.5);
        getPIDController().setOutputRange(-1, 1);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double throttle = Robot.m_oi.getLeftJoystickY();

        if (Robot.vision.isValidTarget()) {
            double limelightAngle = Robot.vision.getTargetX();
            lastLimelightAngle = limelightAngle;
            if(limelightAngle != lastLimelightAngle) {
                double currentNavXAngle = Robot.driveTrain.navX.getAngle();
                double setpoint = currentNavXAngle + lastLimelightAngle;
                getPIDController().setSetpoint(setpoint);
                getPIDController().enable();
            }
        } else {
            getPIDController().disable();
            turn = Robot.m_oi.getRightJoystickX();
        }

        if (Robot.driveTrain.getTalonControlMode() == ControlMode.Velocity)
            Robot.driveTrain.setArcadeDriveVelocity(throttle, turn);
        else
            Robot.driveTrain.setMotorArcadeDrive(throttle, turn);

        //if(Robot.vision.IsTargetGood())
         //   isFinished = true;

    }

    @Override
    protected double returnPIDInput() {
        return Robot.driveTrain.navX.getAngle();
    }

    @Override
    protected void usePIDOutput(double output) {
        turn = -output;
    }

    @Override
    protected boolean isFinished(){
        return getPIDController().onTarget();
    }
    // Called once after isFinished returns true
    @Override
    protected void end() {
        getPIDController().disable();
        Robot.driveTrain.leftAdjustment = 0;
        Robot.driveTrain.rightAdjustment = 0;
        Robot.driveTrain.setDriveMotorsState(true);
        Robot.driveTrain.setMotorArcadeDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }



}
