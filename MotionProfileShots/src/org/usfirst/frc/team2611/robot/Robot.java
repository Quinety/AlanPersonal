package org.usfirst.frc.team2611.robot;

import org.usfirst.frc.team2611.robot.auton.PID;
import org.usfirst.frc.team2611.robot.paths.PathLeft;
import org.usfirst.frc.team2611.robot.paths.PathRight;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	private TalonSRX motorDriveRight, motorDriveLeft, motorDriveLeftFollow, motorDriveRightFollow;
	private Timer m_timer = new Timer();
	private Joystick joyRight, joyLeft;
	private double loops, previousTimer;
	private PID driveRightPID, driveLeftPID, velocityLeft, velocityRight;
	private double leftEncTargetPosition, rightEncTargetPosition, leftEncTargetVelocity, rightEncTargetVelocity, leftEncTargetAccel, rightEncTargetAccel;
	private double largestVelLeft, largestVelRight;
	private double leftEncPosition, rightEncPosition;
	
	@Override
	public void robotInit() {
		motorDriveLeft 			= new TalonSRX(1);
		motorDriveRight			= new TalonSRX(3);
		motorDriveLeftFollow 	= new TalonSRX(0);
		motorDriveRightFollow	= new TalonSRX(2);
		
		motorDriveRightFollow.follow(motorDriveRight);
		motorDriveLeftFollow.follow(motorDriveLeft);
		
		motorDriveLeft.setInverted(true);
		motorDriveLeftFollow.setInverted(true);
		
		joyLeft 	= new Joystick(0);
		joyRight	= new Joystick(1);
		
		driveRightPID 	= new PID(0.0006,0,0);
		driveLeftPID 	= new PID(0.0006,0,0);
		velocityLeft	= new PID(0.00016,0,0);
		velocityRight	= new PID(0.00016,0,0);
		
	}

	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
		
		motorDriveRightFollow.setSelectedSensorPosition(0, 0, 0);
		motorDriveLeftFollow.setSelectedSensorPosition(0,0,0);
		
		loops = 0;
		
		previousTimer = System.currentTimeMillis();
		
		leftEncPosition 	= 0;
		rightEncPosition 	= 0;
		
		leftEncTargetPosition 	= 1;
		rightEncTargetPosition 	= 1;
		
	}

	@Override
	public void autonomousPeriodic() {
		
		double currentTime = System.currentTimeMillis();
		
		if ((currentTime >= previousTimer + 100) || (leftEncPosition > leftEncTargetPosition && rightEncPosition > rightEncTargetPosition) ) {
			loops++;
			previousTimer = currentTime;
		}	
				
		//Gets our velocity that we are currently at
		double leftEncVelocity 	= motorDriveLeftFollow.getSelectedSensorVelocity();
		double rightEncVelocity = motorDriveRightFollow.getSelectedSensorVelocity();
		
		//leftEncPosition = motorDriveLeftFollow.getSelectedSensorPosition();
		//rightEncPosition = motorDriveRightFollow.getSelectedSensorPosition();
		
		SmartDashboard.putNumber("Loops Auton", loops);

		//Our velocity feedforward, 1 over our maximum velocity and our accel feedforward
		double kV = (1 / (((108 / 18.85) * 1440)));
		double kA = 0.000125;
		
		//What the max amount of loops we can do is
		double maxLoops = 205;

		//Gets our target and converts feet to ticks
		if (loops <= maxLoops) {
			leftEncTargetPosition 	= PathLeft.encoderPositionSetpoint(loops) / 18.85 * 1440;
			rightEncTargetPosition 	= PathRight.encoderPositionSetpoint(loops) / 18.85 * 1440;
			
			leftEncTargetVelocity 	= PathLeft.encoderVelSetpoint(loops) / 18.85 * 1440;
			rightEncTargetVelocity 	= PathRight.encoderVelSetpoint(loops) / 18.85 * 1440;
			
			leftEncTargetAccel 	= PathLeft.encoderAccelSetpoint(loops) / 18.85 * 1440;
			rightEncTargetAccel = PathLeft.encoderAccelSetpoint(loops) /18.85 * 1440;
		}
		else {
			leftEncTargetPosition 	= PathLeft.encoderPositionSetpoint(maxLoops) / 18.85 * 1440;
			rightEncTargetPosition  = PathRight.encoderPositionSetpoint(maxLoops) / 18.85 * 1440;
			
			leftEncTargetVelocity 	= 0;
			rightEncTargetVelocity 	= 0;
			
			leftEncTargetAccel 	= 0;
			rightEncTargetAccel = 0;
		}
		
		SmartDashboard.putNumber("Left Vel Target", leftEncTargetVelocity);
		SmartDashboard.putNumber("Right Vel Target", rightEncTargetVelocity);
		
		SmartDashboard.putNumber("LeftFollowEnc", leftEncPosition);
		SmartDashboard.putNumber("RightFollowEnc", rightEncPosition);
		
		double velLeftPIDOutput 	= velocityLeft.PIDInput(leftEncVelocity, leftEncTargetVelocity);
		double velRightPIDOutput 	= velocityRight.PIDInput(rightEncVelocity, rightEncTargetVelocity);
		
		double leftOutput 	= driveLeftPID.PIDInput(leftEncPosition, leftEncTargetPosition);
		double rightOutput 	= driveRightPID.PIDInput(rightEncPosition, rightEncTargetPosition);
		
		//Stops our robot from going too fast or moving backwards
		if (leftOutput < 0) {
			leftOutput = 0;
		}
		if (rightOutput < 0) {
			rightOutput = 0;
		}
		if (leftOutput > 1) {
			leftOutput = 1;
		}
		if (rightOutput > 1) {
			rightOutput = 1;
		}
		
		//Our motor values if we went based on feedback from encoders
		//motorDriveLeft.set(ControlMode.PercentOutput, -leftOutput);
		//motorDriveRight.set(ControlMode.PercentOutput, -rightOutput);
		
		double leftVelFFOutput 	= kV * leftEncTargetVelocity;
		double rightVelFFOutput = kV * rightEncTargetVelocity;
		
		//Feedback velocity no feedforward
		motorDriveLeft.set(ControlMode.PercentOutput, -((leftVelFFOutput) + (velLeftPIDOutput*0)+(kA * leftEncTargetAccel))  );
		motorDriveRight.set(ControlMode.PercentOutput, -((rightVelFFOutput) + (velRightPIDOutput*0)+ (kA * rightEncTargetAccel)) );
		
		//TODO try P * error + D * ((error - prev_error) / dt - goal_velocity) + Kv * goal_velocity + Ka * goal_acceleration, dt will be time difference
	}

	@Override
	public void teleopInit() {
		loops = 0;
		previousTimer = System.currentTimeMillis();
		largestVelLeft = 0;
		largestVelRight = 0;

	}

	@Override
	public void teleopPeriodic() {
		
		double leftVel = motorDriveLeftFollow.getSelectedSensorVelocity();
		double rightVel = motorDriveRightFollow.getSelectedSensorVelocity();
		
		double currentTime = System.currentTimeMillis();
		if (currentTime >= previousTimer + 100) {
			loops++;
			previousTimer = currentTime;
		}
		
		double maxLoops = 205;
		
		if (loops <= maxLoops) {
				leftEncTargetPosition 	= PathLeft.encoderPositionSetpoint(loops) * 12 / 18.85 * 1440;
				rightEncTargetPosition 	= PathRight.encoderPositionSetpoint(loops) * 12 / 18.85 * 1440;
		}
		else {
			leftEncTargetPosition 	= PathLeft.encoderPositionSetpoint(maxLoops)* 12 / 18.85 * 1440;
			rightEncTargetPosition 	= PathRight.encoderPositionSetpoint(maxLoops) * 12 / 18.85 * 1440;
		}
		
		double speedLeft 	= joyLeft.getY();
		double speedRight 	= joyRight.getY();
		
		motorDriveLeft.set(ControlMode.PercentOutput, speedLeft);
		motorDriveRight.set(ControlMode.PercentOutput, speedRight);
		
		if (leftVel > largestVelLeft) {
			largestVelLeft = leftVel;
		}
		
		if (rightVel > largestVelRight) {
			largestVelRight = rightVel;
		}
		
		SmartDashboard.putNumber("LeftFollowEnc", motorDriveLeftFollow.getSelectedSensorPosition());
		SmartDashboard.putNumber("RightFollowEnc", motorDriveRightFollow.getSelectedSensorPosition());
		SmartDashboard.putNumber("Loops", loops);
		SmartDashboard.putNumber("Current Time", currentTime);
		SmartDashboard.putNumber("Left Enc Target", leftEncTargetPosition);
		SmartDashboard.putNumber("Right Enc Target", rightEncTargetPosition);
		SmartDashboard.putNumber("Largest Velocity Left", largestVelLeft);
		SmartDashboard.putNumber("Largest Velocity Right", largestVelRight);
	}

	@Override
	public void testPeriodic() {
	}
}