/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */

/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//getting started
package org.usfirst.frc.team7121.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.TimeUnit;



import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    boolean _lastButton1 = false;
	
	private Joystick m_stick = new Joystick(0);
	private Timer m_timer = new Timer();
    boolean _lastButton3 = false;
    /* save the target position to servo to */
    double targetPositionRotations;
    double targetPositionRotations2;
	private DifferentialDrive m_robotDrive = new DifferentialDrive(new Spark(0), new Spark(1));
    private TalonSRX Arm = new TalonSRX(1);
    private TalonSRX Wrist = new TalonSRX(2);
    private StringBuilder _sb = new StringBuilder();
    private Compressor air;
    private Solenoid s1,s2;
    private DigitalInput forwardLimitSwitch, reverseLimitSwitch;
    private int _loops = 0;
		
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		forwardLimitSwitch = new DigitalInput(1);	//switch top
        reverseLimitSwitch = new DigitalInput(2);  //switch bottom

        // Get the compressor
        air = new Compressor();
        air.start();
        s1 = new Solenoid(1);                        // Solenoid port
        s2 = new Solenoid(2);
        // Get the Joystick
        m_stick = new Joystick(0);
   
        try {
            CameraServer.getInstance().startAutomaticCapture(0);
        } catch (NullPointerException e) {
            System.out.println("Camera 0 not found...skipping.");
        }
        try {
            CameraServer.getInstance().startAutomaticCapture(1);
        } catch (NullPointerException e) {
            System.out.println("Camera 1 not found...skipping.");
        }


	    // when robot starts, leave arm motor off


	    air.setClosedLoopControl(true);
	
	
	

	
				/* choose the sensor and sensor direction */
		Arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx,
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
		Arm.configAllowableClosedloopError(0, 500, Constants.kTimeoutMs);

		/* set closed loop gains in slot0, typically kF stays zero. */
		Arm.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		Arm.config_kP(Constants.kPIDLoopIdx, 0.1, Constants.kTimeoutMs);
		Arm.config_kI(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		Arm.config_kD(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		
		Arm.setNeutralMode(NeutralMode.Brake);
		
		// Zero the Sensor Position
		Arm.setSelectedSensorPosition(0, 0, 10);
		
		Wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* choose to ensure sensor is positive when output is positive */
		//Wrist.setSensorPhase(Constants.kSensorPhase);

		/* choose based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. */ 
		//Wrist.setInverted(Constants.kMotorInvert);

		/* set the peak and nominal outputs, 12V means full */
		Wrist.configNominalOutputForward(0, Constants.kTimeoutMs);
		Wrist.configNominalOutputReverse(0, Constants.kTimeoutMs);
		Wrist.configPeakOutputForward(1, Constants.kTimeoutMs);
		Wrist.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		Wrist.configAllowableClosedloopError(0, 100, Constants.kTimeoutMs);

		/* set closed loop gains in slot0, typically kF stays zero. */
		Wrist.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		Wrist.config_kP(Constants.kPIDLoopIdx, 0.1, Constants.kTimeoutMs);
		Wrist.config_kI(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		Wrist.config_kD(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		
		Wrist.setNeutralMode(NeutralMode.Brake);
		
		// Zero the Sensor Position
		Wrist.setSelectedSensorPosition(0, 0, 10);
	}

	@Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {
	    if (Constants.kGameSpecificMessage.matches("7121")) {
	        String message = DriverStation.getInstance().getGameSpecificMessage();
            // Wait 5ms after asking for the data - this way we don't spam too frequently.
            Timer.delay(0.005);
	        if (message.length() == 3) {
	            Constants.kGameSpecificMessage = message;
	            System.out.println("NEW MESSAGE: " + Constants.kGameSpecificMessage);
            }

        }
    }
		
	

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
		

	}

	/**
	 * This function is called periodically during autonomous.
     * As of this version, ALWAYS SET UP IN FRONT OF THE LEFT SWITCH!
	 */
	@Override
	public void autonomousPeriodic() {
		commonLoop();
	    // Drive for 2 seconds
		if (m_timer.get() < 2.0) {
			m_robotDrive.arcadeDrive(-0.5, 0.0); // drive forwards half speed
		} else {
			m_robotDrive.stopMotor(); // stop robot

            }
            // If the Left switch matches my color, then drop the cube in!
            if (Constants.kGameSpecificMessage.charAt(0) == 'L') {
                // @TODO: Add code to drop in the cube
            	
            	if (m_timer.get() >3&&m_timer.get()<3.2  ) 
            	{	Wrist.set(ControlMode.PercentOutput, 1);
    		}
    	
    		} if  (m_timer.get()>3.2 ) {
    			Wrist.set(ControlMode.PercentOutput, 0);
    			
    			targetPositionRotations = 500000;
    			Arm.set(ControlMode.Position, targetPositionRotations);
    		}
    	
    		if  (m_timer.get()>7.2 ) {
    		
    			
    			targetPositionRotations = 0;
    			Arm.set(ControlMode.Position, targetPositionRotations);
    		}
    	

    			
            	if (m_timer.get()>3.2)
            	{  s1.set(true);
        	    s2.set(false);
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
		teleopLoop();
	}
	private void commonLoop() {
        SmartDashboard.putNumber("SensorVel", Arm.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
        SmartDashboard.putNumber("SensorPos", Arm.getSelectedSensorPosition(Constants.kPIDLoopIdx));
        SmartDashboard.putNumber("MotorOutputPercent", Arm.getMotorOutputPercent());
        SmartDashboard.putNumber("ClosedLoopError", Arm.getClosedLoopError(Constants.kPIDLoopIdx));
        SmartDashboard.putNumber("ClosedLoopTarget", Arm.getClosedLoopTarget(Constants.kPIDLoopIdx));
    }

    private void teleopLoop() {

        m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
	    /* get gamepad axis */
		double rightYstick = m_stick.getRawAxis(5);
		/* calculate the percent motor output */
		double motorOutput = Arm.getMotorOutputPercent();
		boolean ArmUpButton = m_stick.getRawButton(1);
		boolean ArmOverrideButton = m_stick.getRawButton(7);
		boolean ArmDownButton = m_stick.getRawButton(2);

		boolean openGripperButton = m_stick.getRawButton(5);
		boolean closeGripperButton = m_stick.getRawButton(6);
		boolean raiseWristButton = m_stick.getRawButton(4);
		boolean lowerWristButton = m_stick.getRawButton(3);
		boolean raiseArmThenWristButton = false; //@TODO: Pick a button
		boolean lowerWristThenArmButton = false; //@TODO: Pick a button
		boolean scoreSwitch = false; //@TODO: Pick a button
		/* deadband gamepad */
		if (Math.abs(rightYstick) < 0.10) {
			/* within 10% of zero */
			rightYstick = 0;

		}
		
		if (raiseWristButton){
			Wrist.set(ControlMode.PercentOutput, 1);
		}
			else if (lowerWristButton){
				Wrist.set(ControlMode.PercentOutput, -1);
		} else {
			Wrist.set(ControlMode.PercentOutput, 0);
		}
	    /*
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
	
		if (raiseArmThenWristButton) {
			raiseArmThenWrist();
		}
		
		if (lowerWristThenArmButton) {
			lowerArmAndWrist();
		}
		
		if (scoreSwitch) {
			scoreSwitch();
		}

		/* on button1 press enter closed-loop mode on target position */
		if (!_lastButton1 && ArmUpButton) {
			/* Position mode - button just pressed */

			/* 10 Rotations * 4096 u/rev in either direction */
			targetPositionRotations = Constants.kArmHighSetpoint;
			Arm.set(ControlMode.Position, targetPositionRotations);

		}
		if (!_lastButton3 && ArmDownButton) {
			/* Position mode - button just pressed */

			/* 10 Rotations * 4096 u/rev in either direction */
			targetPositionRotations = Constants.kArmLowSetpoint;
			Arm.set(ControlMode.Position, targetPositionRotations);

		}
		/* on button2 just straight drive */
		if (ArmOverrideButton) {
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
		 * print every 100 loops, printing too much too fast is generally bad
		 * for performance
		 */
		if (++_loops >= 100) {
			_loops = 0;
			System.out.println(_sb.toString());
		}
	
		_sb.setLength(0);
		/* save button state for on press detect */
		_lastButton1 = ArmUpButton;
		_lastButton3 = ArmDownButton;
	
		if(openGripperButton) {  //open gripper
            openGripper();
		}
		if(closeGripperButton) {  //close gripper
            closeGripper();
        }
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	private void openGripper() {
	    s1.set(true);
	    s2.set(false);
    }

    private void closeGripper() {
	    s1.set(false);
	    s2.set(true);
    }
    
   
    private void raiseArmThenWrist() {
    	Arm.set(ControlMode.Position, Constants.kArmHighSetpoint);
    	if (Arm.getSelectedSensorPosition(0) > Constants.kArmSafeHeight) {
    		Wrist.set(ControlMode.Position, Constants.kLowWristSetpoint);
    	} else {
    		Wrist.set(ControlMode.Position, Constants.kHighWristSetpoint);
    	}
    }
    
    private void lowerArmAndWrist() {
    	Wrist.set(ControlMode.Position, Constants.kHighWristSetpoint);
    	if (Wrist.getSelectedSensorPosition(0) < Constants.kWristSafePosition) {
    		Arm.set(ControlMode.Position, Constants.kArmLowSetpoint);
    	} else {
    		Arm.set(ControlMode.Position, Constants.kArmHighSetpoint);
    	}
    }
    
    private void scoreSwitch() {
    	Wrist.set(ControlMode.Position, Constants.kMidWristSetpoint);
    	if (Wrist.getClosedLoopError(0) < Constants.kWristAllowableError) {
    		openGripper();
    	} else {
    		closeGripper();
    	}
    	
    }
    }

