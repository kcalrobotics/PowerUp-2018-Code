/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */

/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//getting started
package org.usfirst.frc.team7121.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.TimeUnit;



import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	 
	   
	    private DifferentialDrive m_robotDrive = new DifferentialDrive(new Spark(0), new Spark(1));
	
	private Joystick m_stick = new Joystick(0);
	private Timer m_timer = new Timer();
	TalonSRX Arm = new TalonSRX(1);
	TalonSRX Wrist = new TalonSRX(2);
	
	StringBuilder _sb = new StringBuilder();
	 private Solenoid s1,s2; 
	 DigitalInput forwardLimitSwitch, reverseLimitSwitch;
	 
	
		int _loops = 0;
		boolean _lastButton1 = false;
		boolean _lastButton3 = false;
		
		/** save the target position to servo to */
		double targetPositionRotations;
		double targetPositionRotations2;
		
	 
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		 forwardLimitSwitch = new DigitalInput(1);	//switch top
         reverseLimitSwitch = new DigitalInput(2);  //switch bottom
      
         
        
     
    
         
	Compressor air = new Compressor();
	air.start();
	  s1 = new Solenoid(1);                        // Solenoid port
      s2 = new Solenoid(2);
   

  	CameraServer.getInstance().startAutomaticCapture(0);
	CameraServer.getInstance().startAutomaticCapture(1);


	// when robot starts, leave arm motor off


	air.setClosedLoopControl(true);
	
	
	

	
				/* choose the sensor and sensor direction */
		Arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* choose to ensure sensor is positive when output is positive */
		Arm.setSensorPhase(Constants.kSensorPhase);

		/* choose based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. */ 
		Arm.setInverted(Constants.kMotorInvert);

		/* set the peak and nominal outputs, 12V means full */
		Arm.configNominalOutputForward(0, Constants.kTimeoutMs);
		Arm.configNominalOutputReverse(0, Constants.kTimeoutMs);
		Arm.configPeakOutputForward(1, Constants.kTimeoutMs);
		Arm.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		Arm.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* set closed loop gains in slot0, typically kF stays zero. */
		Arm.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		Arm.config_kP(Constants.kPIDLoopIdx, 0.1, Constants.kTimeoutMs);
		Arm.config_kI(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		Arm.config_kD(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);

		/*
		 * lets grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		int absolutePosition = Arm.getSensorCollection().getPulseWidthPosition();
		/* mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (Constants.kSensorPhase)
			absolutePosition *= -1;
		if (Constants.kMotorInvert)
			absolutePosition *= -1;
		/* set the quadrature (relative) sensor to match absolute */
		Arm.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}
		
	

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
		
		m_stick = new Joystick(0);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		if (m_timer.get() < 2.0) {
			m_robotDrive.arcadeDrive(-0.5, 0.0); // drive forwards half speed
		} else {
			m_robotDrive.stopMotor(); // stop robot
		}
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		
		commonLoop();
	}
	void commonLoop() {
		
	m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
		
		SmartDashboard.putNumber("SensorVel", Arm.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
		SmartDashboard.putNumber("SensorPos",Arm.getSelectedSensorPosition(Constants.kPIDLoopIdx));
		SmartDashboard.putNumber("MotorOutputPercent", Arm.getMotorOutputPercent());
		SmartDashboard.putNumber("ClosedLoopError", Arm.getClosedLoopError(Constants.kPIDLoopIdx));
		SmartDashboard.putNumber("ClosedLoopTarget", Arm.getClosedLoopTarget(Constants.kPIDLoopIdx));
		/* get gamepad axis */
		double rightYstick = m_stick.getRawAxis(5);
		/* calculate the percent motor output */
		double motorOutput = Arm.getMotorOutputPercent();
		boolean button1 = m_stick.getRawButton(1);
		boolean button2 = m_stick.getRawButton(7);
		boolean button3 = m_stick.getRawButton(2);
		/* deadband gamepad */
		if (Math.abs(rightYstick) < 0.10) {
			/* within 10% of zero */
			rightYstick = 0;

		}

		
		//commit out the button control for wrist
		
		if (m_stick.getRawButton(4) == true){
			Wrist.set(ControlMode.PercentOutput, 1);
		}
			else if (m_stick.getRawButton(3) == true){
				Wrist.set(ControlMode.PercentOutput, -1);
		} else {
			Wrist.set(ControlMode.PercentOutput, 0);
		}
	/**
		// run arm motor 
				if (m_stick.getRawButton(4) == true )  //&& forwardLimitSwitch.get()==false
				{
					Arm.set(1.0);
				}
					else if (m_stick.getRawButton(6) == true )//&& reverseLimitSwitch.get()==false
					{
						Arm.set(-1.0);
				} else {
					Arm.set(0.0);
				}
				
				*/
		
		/* get gamepad axis - forward stick is positive */
		
		
		
		
	
	
		/* prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(Arm.getSelectedSensorVelocity(Constants.kPIDLoopIdx));

		
		/* prepare line to print */
		_sb.append("\tout:");
		/* cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%"); /* perc */

		_sb.append("\tpos:");
		_sb.append(Arm.getSelectedSensorPosition(0));
		_sb.append("u"); /* units */

		/* on button1 press enter closed-loop mode on target position */
		if (!_lastButton1 && button1) {
			/* Position mode - button just pressed */

			/* 10 Rotations * 4096 u/rev in either direction */
			targetPositionRotations = 0;
			Arm.set(ControlMode.Position, targetPositionRotations);

		}
		if (!_lastButton3 && button3) {
			/* Position mode - button just pressed */

			/* 10 Rotations * 4096 u/rev in either direction */
			targetPositionRotations = 5000;
			Arm.set(ControlMode.Position, targetPositionRotations);

		}
		/* on button2 just straight drive */
		if (button2) {
			/* Percent voltage mode */
			Arm.set(ControlMode.PercentOutput, rightYstick);
		}
		/* if Talon is in position closed-loop, print some more info */
		if (Arm.getControlMode() == ControlMode.Position) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(Arm.getClosedLoopError(0));
			_sb.append("u"); /* units */

			_sb.append("\ttrg:");
			_sb.append(targetPositionRotations);
			_sb.append("u"); /* units */
		}
		/*
		 * print every ten loops, printing too much too fast is generally bad
		 * for performance
		 */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
		}
	
		_sb.setLength(0);
		/* save button state for on press detect */
		_lastButton1 = button1;
		_lastButton3 = button3;
	
	

				
				 if(m_stick.getRawButton(5) == true)  //open gripper
			        {
			              s1.set(true);
			              s2.set(false);
			         }
			         if(m_stick.getRawButton(6) == true)  //close gripper
			         {
			              s1.set(false);
			              s2.set(true);
			          }
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
