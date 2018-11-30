package org.usfirst.frc.team2611.robot.teleop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

public class Shooterupper {
	
	private static boolean button_shooter_previous, button_shooter_current;
	
	private static double time_current, motor_shooter_start_time;
	
	private static TalonSRX motor_shooterupper;
	
	private static TalonSRX motor_setterupper;

	private static Joystick gamepad;

	public static void run() {
		
		//Running the shooter mechanism
		button_shooter_previous = button_shooter_current;
		
		button_shooter_current = gamepad.getRawButton(1);
		
		if (button_shooter_current) {
			if (!button_shooter_previous) {
				motor_shooter_start_time = time_current;
			}
			//motor_shooterupper.set(ControlMode.PercentOutput, -.5);
			motor_shooterupper.set(ControlMode.Velocity, -100);
			if (motor_shooter_start_time + 1 < time_current) {
				motor_setterupper.set(ControlMode.PercentOutput, -.5);
			}
		}
		else {
			motor_shooterupper.set(ControlMode.Velocity, 0);
			motor_setterupper.set(ControlMode.PercentOutput, 0);
		}
		
	}

}
