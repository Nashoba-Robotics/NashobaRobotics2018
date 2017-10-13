package edu.nr.lib.driving;

import edu.nr.lib.NRMath;

public class DriveTypeCalculations {
	
	public DriveTypeCalculations(double highTurnNonLinearity, double lowTurnNonLinearity, double highNegIntertiaScalar,
			double lowNegInertiaThreshold, double lowNegInertiaTurnScalar, double lowNegInertiaCloseScalar, double lowNegInertiaFarScalar,
			double highSensitivity, double lowSensitivity, double quickStopDeadband, double quickStopWeight, double quickStopScalar) {
			//Create doubles above and assign them here
	}
	
	/**
	 * Adapted from 254's CheesyDriveHelper
	 * 
	 * @param moveRaw
	 * 				The -1 to 1 (back to front) joystick input
	 * @param turnRaw
	 * 				The -1 to 1 (left to right) joystick input
	 * @param isQuickTurn
	 * 				Whether quick turn button is depressed
	 * @param isHighGear
	 * 				Whether in high or low gear; low gear default if no high gear
	 * @return
	 * 				Array of doubles with left motor percent followed by the right motor percent
	 */
	public double[] cheesyDrive(double moveRaw, double turnRaw, boolean isQuickTurn, boolean isHighGear) {
		
		double oldTurn = 0;
		double quickStopAccumulator = 0;
		double negInertiaAccumulator = 0;
		
		double leftMotorPercent = 0, rightMotorPercent = 0;
		double move = NRMath.limit(moveRaw);
		double turn = NRMath.limit(turnRaw);
		
		return new double[] {leftMotorPercent, rightMotorPercent};
	}
	
	/**
	 * Sets left and right motor speeds to the speeds needed for the given move
	 * and turn values
	 * 
	 * @param move
	 *            The speed, from -1 to 1 (inclusive), that the robot should go
	 *            at. 1 is max forward, 0 is stopped, -1 is max backward
	 * @param turn
	 *            The speed, from -1 to 1 (inclusive), that the robot should
	 *            turn at. 1 is max right, 0 is stopped, -1 is max left
	 * @return 
	 * 			Array of doubles of left motor percent followed by right motor percent
	 */
	public double[] arcadeDrive(double moveRaw, double turnRaw) {
		double move = NRMath.limit(moveRaw);
		double turn = NRMath.limit(turnRaw);
		double leftMotorPercent, rightMotorPercent;

		if (move > 0.0) {
			if (turn > 0.0) {
				leftMotorPercent = move - turn;
				rightMotorPercent = Math.max(move, turn);
			} else {
				leftMotorPercent = Math.max(move, -turn);
				rightMotorPercent = move + turn;
			}
		} else {
			if (turn > 0.0) {
				leftMotorPercent = -Math.max(-move, turn);
				rightMotorPercent = move + turn;
			} else {
				leftMotorPercent = move - turn;
				rightMotorPercent = -Math.max(-move, -turn);
			}
		}
		
		return new double[] {leftMotorPercent, rightMotorPercent};
	}
}
