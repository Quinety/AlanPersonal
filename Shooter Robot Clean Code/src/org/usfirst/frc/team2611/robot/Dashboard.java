package org.usfirst.frc.team2611.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {
	
	private static int encoder_shooter;
	protected static int encoder_drive_left;
	protected static int encoder_drive_right;
	private static AHRS gyro;
	
	public static void display(){
		
		
			SmartDashboard.putNumber("Motor Left Drive Position", encoder_drive_left);
			SmartDashboard.putNumber("Motor Right Drive Position", encoder_drive_right);
			SmartDashboard.putNumber("Shooter Encoder Velocity", encoder_shooter);
			SmartDashboard.putNumber("Gyro", gyro.getYaw());
		
		
	}

}
