/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2611.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class Robot extends IterativeRobot {
	private Timer m_timer = new Timer();
	private Joystick hid_left, hid_right, gamepad;
	private TalonSRX motor_drive_right, motor_drive_left, motor_drive_left_follower, motor_drive_right_follower, motor_elevator,
	motor_actuator, motor_actuator_follower, motor_intake_left, motor_intake_right;
	private double speed_right, speed_left;

	@Override
	public void robotInit() {
		motor_drive_right 			= new TalonSRX(3);
		motor_drive_right_follower 	= new TalonSRX(2);
		motor_drive_left 			= new TalonSRX(0);
		motor_drive_left_follower 	= new TalonSRX(1);
		
		motor_actuator				= new TalonSRX(4);
		motor_actuator_follower 	= new TalonSRX(5);
		motor_actuator_follower.setInverted(true);
		motor_actuator_follower.follow(motor_actuator);
		
		motor_elevator 				= new TalonSRX(6);
		motor_intake_left 			= new TalonSRX(8);
		motor_intake_right			= new TalonSRX(7);
		
		motor_drive_left_follower.follow(motor_drive_left);
		motor_drive_right_follower.follow(motor_drive_right);
		
		motor_drive_left.setInverted(true);
		motor_drive_left_follower.setInverted(true);

		hid_left 	= new Joystick(0);
		hid_right 	= new Joystick(1);
		gamepad 	= new Joystick(2);
		
        new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(640, 480);
            
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
            
            Mat source = new Mat();
            Mat output = new Mat();
            
            while(!Thread.interrupted()) {
                cvSink.grabFrame(source);
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
            }
        }).start();
		
	}

	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {
		speed_right = hid_right.getRawAxis(1);
		speed_left = (hid_left.getRawAxis(1));
		motor_drive_left.set(ControlMode.PercentOutput, speed_left);
		motor_drive_right.set(ControlMode.PercentOutput, speed_right);
		
		double intake_left_value = gamepad.getRawAxis(1);
		double intake_right_value = gamepad.getRawAxis(5);
		
		motor_intake_left.set(ControlMode.PercentOutput, intake_left_value);
		motor_intake_right.set(ControlMode.PercentOutput, intake_right_value);
		
		double elevator_value = (gamepad.getRawAxis(2)) - (gamepad.getRawAxis(3));
		
		motor_elevator.set(ControlMode.PercentOutput, elevator_value);
		

		
		if (gamepad.getRawButton(5)) {
			motor_actuator.set(ControlMode.PercentOutput, .4);
		}
		else if (gamepad.getRawButton(6)) {
			motor_actuator.set(ControlMode.PercentOutput, -.4);
		}
		else {
			motor_actuator.set(ControlMode.PercentOutput, 0);
		}
		
		if (gamepad.getRawButton(5) && gamepad.getRawButton(6)) {
			motor_actuator.set(ControlMode.PercentOutput, 0);
			}

	}

	@Override
	public void testPeriodic() {
	}
}
