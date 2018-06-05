package edu.nr.lib.driving;

import edu.nr.lib.NRMath;

public class DriveTypeCalculations {
	
	private static double highNegInertiaThreshold;
	private static double highNegInertiaTurnScalar;
	private static double highNegInertiaCloseScalar;
	private static double highNegInertiaFarScalar;
	private static double lowNegInertiaThreshold;
	private static double lowNegInertiaTurnScalar;
	private static double lowNegInertiaCloseScalar;
	private static double lowNegInertiaFarScalar;
	
	public DriveTypeCalculations(double highNegInertiaThreshold_, double highNegInertiaTurnScalar_, double highNegInertiaCloseScalar_, double highNegInertiaFarScalar_, double lowNegInertiaThreshold_, double lowNegInertiaTurnScalar_, double lowNegInertiaCloseScalar_, double lowNegInertiaFarScalar_) {
			
		highNegInertiaThreshold = highNegInertiaThreshold_;
		highNegInertiaTurnScalar = highNegInertiaTurnScalar_;
		highNegInertiaCloseScalar = highNegInertiaCloseScalar_;
		highNegInertiaFarScalar = highNegInertiaFarScalar_;
		lowNegInertiaThreshold = lowNegInertiaThreshold_;
		lowNegInertiaTurnScalar = lowNegInertiaTurnScalar_;
		lowNegInertiaCloseScalar = lowNegInertiaThreshold_;
		lowNegInertiaFarScalar = lowNegInertiaFarScalar_;
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
	public static double[] cheesyDrive(double moveRaw, double turnRaw, double oldTurnRaw, boolean isHighGear) {
				
		double negInertiaAccumulator = 0;
		
		double move = NRMath.limit(moveRaw);
		double turn = NRMath.limit(turnRaw);
		double oldTurn = NRMath.limit(oldTurnRaw);
		
		double negInertia = turn - oldTurn;
        oldTurn = turn;
		
        // Negative inertia!
        double negInertiaScalar;
        if (turn * negInertia > 0) {
            // If we are moving away from 0.0, aka, trying to get more wheel.
            if(isHighGear) {
            	negInertiaScalar = highNegInertiaTurnScalar;
            } else {
            	negInertiaScalar = lowNegInertiaTurnScalar;
            }
            
        } else {
            // Otherwise, we are attempting to go back to 0.0.
            if (isHighGear) {
            	if (Math.abs(turn) > highNegInertiaThreshold) {
            		negInertiaScalar = highNegInertiaFarScalar;
            	} else {
            		negInertiaScalar = highNegInertiaCloseScalar;
            	}
            }
            else {
            	if (Math.abs(turn) > lowNegInertiaThreshold) {
            		negInertiaScalar = lowNegInertiaFarScalar;
            	} else {
            		negInertiaScalar = lowNegInertiaCloseScalar;
            	}
            }
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
        
        return arcadeDrive(move, turn);
        
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
