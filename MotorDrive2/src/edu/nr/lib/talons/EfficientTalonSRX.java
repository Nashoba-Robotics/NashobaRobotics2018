package edu.nr.lib.talons;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
