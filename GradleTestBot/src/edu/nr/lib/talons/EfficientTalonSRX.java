package edu.nr.lib.talons;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Adapted from 254's LazyCANTalon
 * 
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */

public class EfficientTalonSRX extends TalonSRX {

	protected double mLastSet = 0;
    protected ControlMode mLastControlMode = ControlMode.PercentOutput;
    
    /**
     * Creates new EfficientTalonSRX
     * @param deviceNumber
     */
    public EfficientTalonSRX(int deviceNumber) {
        super(deviceNumber);
    }
    
    @Override
    public void set(ControlMode controlMode, double value) {
    	if (value != mLastSet || controlMode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = controlMode;
            super.set(mLastControlMode, mLastSet);
    	}
    }
}
