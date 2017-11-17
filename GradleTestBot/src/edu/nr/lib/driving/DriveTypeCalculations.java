package edu.nr.lib.driving;

import edu.nr.lib.NRMath;

public class DriveTypeCalculations {
	
	private static double highTurnNonLinearity;
	private static double lowTurnNonLinearity;
	private static double highNegInertiaScalar;
	private static double lowNegInertiaThreshold;
	private static double lowNegInertiaTurnScalar;
	private static double lowNegInertiaCloseScalar;
	private static double lowNegInertiaFarScalar;
	private static double highSensitivity;
	private static double lowSensitivity;
	private static double quickStopDeadband;
	private static double quickStopWeight;
	private static double quickStopScalar;
	
	
	public DriveTypeCalculations(double highTurnNonLinearity, double lowTurnNonLinearity, double highNegInertiaScalar,
			double lowNegInertiaThreshold, double lowNegInertiaTurnScalar, double lowNegInertiaCloseScalar, double lowNegInertiaFarScalar,
			double highSensitivity, double lowSensitivity, double quickStopDeadband, double quickStopWeight, double quickStopScalar) {
			
			DriveTypeCalculations.highTurnNonLinearity = highTurnNonLinearity;
			DriveTypeCalculations.lowTurnNonLinearity = lowTurnNonLinearity;
			DriveTypeCalculations.highNegInertiaScalar = highNegInertiaScalar;
			DriveTypeCalculations.lowNegInertiaTurnScalar = lowNegInertiaTurnScalar;
			DriveTypeCalculations.lowNegInertiaCloseScalar = lowNegInertiaThreshold;
			DriveTypeCalculations.lowNegInertiaFarScalar = lowNegInertiaFarScalar;
			DriveTypeCalculations.highSensitivity = highSensitivity;
			DriveTypeCalculations.lowSensitivity = lowSensitivity;
			DriveTypeCalculations.quickStopDeadband = quickStopDeadband;
			DriveTypeCalculations.quickStopWeight = quickStopWeight;
			DriveTypeCalculations.quickStopScalar = quickStopScalar;
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
	public static double[] cheesyDrive(double moveRaw, double turnRaw, boolean isQuickTurn, boolean isHighGear) {
		
		double oldTurn = 0;
		double quickStopAccumulator = 0;
		double negInertiaAccumulator = 0;
		
		double move = NRMath.limit(moveRaw);
		double turn = -NRMath.limit(turnRaw);
		
		double negInertia = turn - oldTurn;
        oldTurn = turn;
        
        double turnNonLinearity;
        if (isHighGear) {
            turnNonLinearity = highTurnNonLinearity;
            final double denominator = Math.sin(Math.PI / 2.0 * turnNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            turn = Math.sin(Math.PI / 2.0 * turnNonLinearity * turn) / denominator;
            turn = Math.sin(Math.PI / 2.0 * turnNonLinearity * turn) / denominator;
        } else {
            turnNonLinearity = lowTurnNonLinearity;
            final double denominator = Math.sin(Math.PI / 2.0 * turnNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            turn = Math.sin(Math.PI / 2.0 * turnNonLinearity * turn) / denominator;
            turn = Math.sin(Math.PI / 2.0 * turnNonLinearity * turn) / denominator;
            turn = Math.sin(Math.PI / 2.0 * turnNonLinearity * turn) / denominator;
        }
        
        double leftMotorPercent, rightMotorPercent, overPower;
        double sensitivity;

        double angularPower;
        double linearPower;
		
        // Negative inertia!
        double negInertiaScalar;
        if (isHighGear) {
            negInertiaScalar = highNegInertiaScalar;
            sensitivity = highSensitivity;
        } else {
            if (turn * negInertia > 0) {
                // If we are moving away from 0.0, aka, trying to get more wheel.
                negInertiaScalar = lowNegInertiaTurnScalar;
            } else {
                // Otherwise, we are attempting to go back to 0.0.
                if (Math.abs(turn) > lowNegInertiaThreshold) {
                    negInertiaScalar = lowNegInertiaFarScalar;
                } else {
                    negInertiaScalar = lowNegInertiaCloseScalar;
                }
            }
            sensitivity = lowSensitivity;
        }
        double negInertiaPower = negInertia * negInertiaScalar;
        negInertiaAccumulator += negInertiaPower;

        turn = turn + negInertiaAccumulator;
        if (negInertiaAccumulator > 1) {
            negInertiaAccumulator -= 1;
        } else if (negInertiaAccumulator < -1) {
            negInertiaAccumulator += 1;
        } else {
            negInertiaAccumulator = 0;
        }
        linearPower = move;
        
        // Quickturn!
        if (isQuickTurn) {
            if (Math.abs(linearPower) < quickStopDeadband) {
                double alpha = quickStopWeight;
                quickStopAccumulator = (1 - alpha) * quickStopAccumulator
                        + alpha * NRMath.limit(turn, 1.0) * quickStopScalar;
            }
            overPower = 1.0;
            angularPower = turn;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(move) * turn * sensitivity - quickStopAccumulator;
            if (quickStopAccumulator > 1) {
                quickStopAccumulator -= 1;
            } else if (quickStopAccumulator < -1) {
                quickStopAccumulator += 1;
            } else {
                quickStopAccumulator = 0.0;
            }
        }
        
        rightMotorPercent = leftMotorPercent = linearPower;
        leftMotorPercent += angularPower;
        rightMotorPercent -= angularPower;
        
        if (leftMotorPercent > 1.0) {
            rightMotorPercent -= overPower * (leftMotorPercent - 1.0);
            leftMotorPercent = 1.0;
        } else if (rightMotorPercent > 1.0) {
            leftMotorPercent -= overPower * (rightMotorPercent - 1.0);
            rightMotorPercent = 1.0;
        } else if (leftMotorPercent < -1.0) {
            rightMotorPercent += overPower * (-1.0 - leftMotorPercent);
            leftMotorPercent = -1.0;
        } else if (rightMotorPercent < -1.0) {
            leftMotorPercent += overPower * (-1.0 - rightMotorPercent);
            rightMotorPercent = -1.0;
        }
        
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
	public static double[] arcadeDrive(double moveRaw, double turnRaw) {
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
