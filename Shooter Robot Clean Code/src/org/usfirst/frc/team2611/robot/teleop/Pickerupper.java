package org.usfirst.frc.team2611.robot.teleop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

public class Pickerupper {
	
	private static boolean button_toggle_feeder_previous, button_toggle_feeder_current, motor_feeder_enable;
	private static Joystick hid_right;
	public static TalonSRX motor_pickerupper;
	
	public static void run() {
		
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
		
	}
}
