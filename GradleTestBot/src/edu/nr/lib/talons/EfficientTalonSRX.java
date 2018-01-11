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

	protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;
    
    /**
     * Creates new EfficientTalonSRX
     * @param deviceNumber
     */
    public EfficientTalonSRX(int deviceNumber) {
        super(deviceNumber);
    }
    
    public void set(double value) {
    	set(getControlMode(), value);
    }
    
    public void set(ControlMode controlMode) {
    	set(controlMode, 0);
    }
    
    @Override
    public void set(ControlMode controlMode, double value) {
    	if (value != mLastSet || getControlMode() != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = getControlMode();
            super.set(mLastControlMode, mLastSet);
        }
    }
}
