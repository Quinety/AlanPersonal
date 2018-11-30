package org.usfirst.frc.team2611.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class MotorSetup {
	
	protected static TalonSRX	motor_drive_right;
	protected static TalonSRX motor_drive_left;
	private static TalonSRX motor_drive_right_follower;
	private static TalonSRX motor_drive_left_follower;
	private static TalonSRX motor_pickerupper;
	private static TalonSRX motor_setterupper;
	private static TalonSRX motor_shooterupper;
	private static TalonSRX motor_shooterupper_follower;
	
	public static void Setup() {
		
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
		
		//Pid for shooterupper
		//PID values
		motor_shooterupper.config_kP(0, 4, 100);
		motor_shooterupper.config_kI(0, 0.01, 100);
		motor_shooterupper.config_kD(0, 0, 100);
		motor_shooterupper.setSensorPhase(true);
		
	}

}
