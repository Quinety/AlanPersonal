package org.usfirst.frc.team2611.robot;

import org.usfirst.frc.team2611.robot.auton.PID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	protected static TalonSRX	motor_drive_right, motor_drive_left;
	public static TalonSRX motor_pickerupper, motor_setterupper, motor_shooterupper;
	private Joystick hid_left, hid_right, gamepad;
	private double time_current, motor_shooter_start_time;
	private boolean  button_toggle_feeder_previous, button_toggle_feeder_current, motor_feeder_enable, button_shooter_previous, button_shooter_current;
	private int encoder_shooter, encoder_drive_left, encoder_drive_right;
	private static AHRS gyro;
	private static TalonSRX motor_drive_right_follower, motor_drive_left_follower, motor_shooterupper_follower;
	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry tx = table.getEntry("tx");
	NetworkTableEntry ta = table.getEntry("ta");
	NetworkTableEntry tv = table.getEntry("tv");
	private double leftOutput, rightOutput;

	//calls my pid
	public static PID turn, dis, shooter, yAxis, xAxis;
		
	private Timer m_timer = new Timer();

	@Override
	public void robotInit() {
		
		//Sets up drivebase motors
		motor_drive_right				= new TalonSRX(0);
		motor_drive_left				= new TalonSRX(2);
		motor_drive_right_follower		= new TalonSRX(1);
		motor_drive_left_follower		= new TalonSRX(3);
		//Sets up the followers for the drivebase
		motor_drive_right_follower.follow(motor_drive_right);
		motor_drive_left_follower.follow(motor_drive_left);		
		
		//Sets the TalonSRX ID's to the pickerupper and the setterupper
		motor_pickerupper 				= new TalonSRX(4);
		motor_setterupper 				= new TalonSRX(5);
		//Shooterupper and follower
		motor_shooterupper 				= new TalonSRX(6);
		motor_shooterupper_follower 	= new TalonSRX(7);
		motor_shooterupper_follower.follow(motor_shooterupper);
		
		motor_shooterupper.setInverted(true);
		motor_shooterupper.setSensorPhase(true);
		
		//Configs the pid values for the shooter
		motor_shooterupper.config_kP(0, 1, 0);
		motor_shooterupper.config_kI(0, 0, 0);
		motor_shooterupper.config_kD(0, 0, 0);
		motor_shooterupper.config_kF(0, 0, 0);
		
		//Pickerupper Toggle Variables
		button_toggle_feeder_previous 	= false;
		motor_feeder_enable 			= false;
		
		//Shooterupper and Setterupper Vars
		button_shooter_current = false;
		
		//Sets up our joysticks for teleop
		hid_left	= new Joystick(0);
		hid_right	= new Joystick(1);
		gamepad 	= new Joystick(2);
		
		//Sets up the navx
		gyro = new AHRS(SPI.Port.kMXP);
		
		//Sets up the Pid with values too
		turn = new PID(.052, 0, 0);
		dis = new PID(.00022, 0, 0);
		shooter = new PID(.004, 0, 0);
		yAxis = new PID(0, 0, 0);
		xAxis = new PID(0, 0, 0);

	}

	@Override
	public void autonomousInit() {
		
		m_timer.reset();
		m_timer.start();
		
	}

	@Override
	public void autonomousPeriodic() {
		
		double isTarget = tv.getDouble(0);
		
		if (isTarget == 1) {
			
			double maxOutput = .7;
			
			//Gets our distance from the target and our angle 
			double distance = Math.sqrt(5776/ (ta.getDouble(0))) * 76.39;
			double angle = tx.getDouble(0);
			
			//What we should do based on our distance and angle through our pid
			double disOutput = dis.PIDInput(distance, 5000);
			double angleOutput = turn.PIDInput(angle, 1);
			
			//Dumb smartboard values
			SmartDashboard.putNumber("PID Distance Output", disOutput);
			SmartDashboard.putNumber("PID Angle Output", angleOutput);
			SmartDashboard.putNumber("Distance", distance);
			SmartDashboard.putNumber("Angle", angle);
			
			//Tells robot which way to turn
			if (angle > 0) {
				leftOutput = -(disOutput);
				rightOutput = (disOutput - angleOutput);
			}
			if (angle < 0) {
				leftOutput = -(disOutput + angleOutput);
				rightOutput = (disOutput);
			}
			
			//Makes sure robot isn't going too fast
			if (leftOutput > maxOutput) {
				leftOutput = maxOutput;
			}
			else if (leftOutput < -maxOutput) {
				leftOutput = -maxOutput;
			}
			
			if (rightOutput > maxOutput) {
				rightOutput = maxOutput;
			}
			else if (rightOutput < -maxOutput) {
				rightOutput = -maxOutput;
			}
			
			//Sets robot to our speed
			motor_drive_left.set(ControlMode.PercentOutput, leftOutput);
			motor_drive_right.set(ControlMode.PercentOutput, rightOutput);
		}
		else {
			
			motor_drive_left.set(ControlMode.PercentOutput, 0);
			motor_drive_right.set(ControlMode.PercentOutput, 0);
		}
	}

	@Override
	public void teleopInit() {

	}

	@Override
	public void teleopPeriodic() {

		//Gets our values for our encoders
		encoder_shooter = motor_shooterupper.getSelectedSensorPosition(0);
		encoder_drive_left = motor_drive_left.getSelectedSensorPosition(0);
		encoder_drive_right = motor_drive_right.getSelectedSensorPosition(0);
		
		//Dashboard Junk
		SmartDashboard.putNumber("Motor Left Drive Position", encoder_drive_left);
		SmartDashboard.putNumber("Motor Right Drive Position", encoder_drive_right);
		SmartDashboard.putNumber("Shooter Encoder Position", encoder_shooter);
		SmartDashboard.putNumber("Shooter Velocity", motor_shooterupper.getSelectedSensorVelocity());
		SmartDashboard.putNumber("Gyro", gyro.getYaw());
		SmartDashboard.putNumber("Area Limelight", ta.getDouble(0));
		SmartDashboard.putNumber("XCords Limelight", tx.getDouble(0));
		SmartDashboard.putNumber("Gyro Accel X", gyro.getRawAccelX());
		SmartDashboard.putNumber("Gyro Accel Z", gyro.getRawAccelZ());
		SmartDashboard.putNumber("Gyro Accel Y", gyro.getRawAccelY());

		// Compute speed output and sends to motors
		double temporary = hid_right.getY();
		double speed_right = (Math.cos(Math.PI * temporary) - 1) / 2;
		
		if ( temporary > 0 ) speed_right *= -1;
		temporary = 0 - hid_left.getY();
		double speed_left = (Math.cos(Math.PI * temporary) - 1) / 2;
		
		if ( temporary > 0 ) speed_left *= -1;
		
		motor_drive_left.set(ControlMode.PercentOutput, speed_left);
		motor_drive_right.set(ControlMode.PercentOutput, speed_right);
		

		//does the pickerupper
		button_toggle_feeder_current = hid_right.getRawButton(1);
		if (button_toggle_feeder_current && !button_toggle_feeder_previous) {
			motor_feeder_enable = !motor_feeder_enable;
		}
		
		button_toggle_feeder_previous = button_toggle_feeder_current;
		
		if (motor_feeder_enable) {
			motor_pickerupper.set(ControlMode.PercentOutput, -.85);
		}
		else {
			motor_pickerupper.set(ControlMode.PercentOutput, 0);
		}
		
		//Running the shooter mechanism
		button_shooter_previous = button_shooter_current;
		button_shooter_current = gamepad.getRawButton(1);
		
		if (button_shooter_current) {
			if (!button_shooter_previous) {
				motor_shooter_start_time = time_current;
			}
			motor_shooterupper.set(ControlMode.Velocity, -100);
			if (motor_shooter_start_time + 1 < time_current) {
				motor_setterupper.set(ControlMode.PercentOutput, -.5);
			}
		}
		else {
			
			motor_shooterupper.set(ControlMode.PercentOutput, 0);
			motor_setterupper.set(ControlMode.PercentOutput, 0);
		}
		
		if (hid_right.getRawButton(6)) {
			
			double upDownVel 			= 0;
			double upDownAccel 			= 0;
			double leftRightVel 		= 0;
			double leftRightAccel 		= 0;
			double time 				= 0;
			double yOutput 				= 0;
			double xOutput				= 0;
				
			boolean isDone = false;
			
			gyro.reset();
			m_timer.reset();
			double startTime = System.nanoTime();

			while (!isDone) {
				
				time = (System.nanoTime() - startTime)/ 1000000000;
				
				upDownAccel = gyro.getRawAccelY();
				
				leftRightAccel = gyro.getRawAccelX();
				
				//yOutput = yAxis.PIDInput(, 1000);
				//xOutput = xAxis.PIDInput(, 0);

				upDownVel = upDownVel + (upDownAccel * time);
				leftRightVel = leftRightVel + (leftRightAccel * time);
				
			}
		}
	}

	@Override
	public void testPeriodic() {
	}
}
