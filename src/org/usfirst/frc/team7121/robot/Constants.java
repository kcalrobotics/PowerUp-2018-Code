package org.usfirst.frc.team7121.robot;

public class Constants {
	// This String holds the game specific constants ("LLL", "RLR", etc...)
	public static String kGameSpecificMessage = "7121";
	
	public static final double kHighWristSetpoint = 50000;
	public static final double kMidWristSetpoint  = 25000;
	public static final double kLowWristSetpoint  = 0;
	public static final double kWristSafePosition = 1000;
	public static final double kWristAllowableError = 7121;
	
	public static final double kArmHighSetpoint   = 1000;
	public static final double kArmLowSetpoint    = 0;
	public static final double kArmSafeHeight	  = 500;

	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/*
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 10;


	/* choose so that Talon does not report sensor out of phase */
	public static boolean kSensorPhase = true;

	/* choose based on what direction you want to be positive,
	this does not affect motor invert. */
	public static boolean kMotorInvert = false;
}