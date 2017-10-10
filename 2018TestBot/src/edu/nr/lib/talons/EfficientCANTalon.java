package edu.nr.lib.talons;

import com.ctre.CANTalon;

/**
 * Adapted from 254's LazyCANTalon
 * 
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */

public class EfficientCANTalon extends CANTalon {

	protected double mLastSet = Double.NaN;
    protected TalonControlMode mLastControlMode = null;
    
    /**
     * Creates new EfficientCANTalon
     * @param deviceNumber
     * @param controlPeriodMs
     * @param enablePeriodMs
     */
    public EfficientCANTalon(int deviceNumber, int controlPeriodMs, int enablePeriodMs) {
        super(deviceNumber, controlPeriodMs, enablePeriodMs);
    }
    
    /**
     * Creates new EfficientCANTalon
     * @param deviceNumber
     * @param controlPeriodMs
     */
    public EfficientCANTalon(int deviceNumber, int controlPeriodMs) {
        super(deviceNumber, controlPeriodMs);
    }
    
    /**
     * Createws new EfficientCANTalon
     * @param deviceNumber
     */
    public EfficientCANTalon(int deviceNumber) {
        super(deviceNumber);
    }
    
    @Override
    public void set(double value) {
        if (value != mLastSet || getControlMode() != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = getControlMode();
            super.set(value);
        }
    }
}
