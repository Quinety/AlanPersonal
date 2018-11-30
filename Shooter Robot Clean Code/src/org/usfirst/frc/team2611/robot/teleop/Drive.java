package org.usfirst.frc.team2611.robot.teleop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

public class Drive {
	
	protected static TalonSRX motor_drive_right;
	protected static TalonSRX motor_drive_left;
	private static double speed_right;
	private static double speed_left;
	private static double temporary;
	private static Joystick hid_left;
	private static Joystick hid_right;
	
	public static void run() {
		// Compute speed output
		temporary = hid_right.getY();
		speed_right = (Math.cos(Math.PI * temporary) - 1) / 2;
		
		if ( temporary > 0 ) speed_right *= -1;
		temporary = 0 - hid_left.getY();
		speed_left = (Math.cos(Math.PI * temporary) - 1) / 2;
		
		if ( temporary > 0 ) speed_left *= -1;
		
		motor_drive_left.set(ControlMode.PercentOutput, speed_left);
		motor_drive_right.set(ControlMode.PercentOutput, speed_right);
		
	}

}
