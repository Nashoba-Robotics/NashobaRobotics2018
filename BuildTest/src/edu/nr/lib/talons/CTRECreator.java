package edu.nr.lib.talons;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_ControlFrame;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.MotorSafety;

/**
 * Adapted from 254's CANTalonFactory
 * 
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */

public class CTRECreator {

	public static class Configuration {
        public LimitSwitchSource LIMIT_SWITCH_SOURCE = LimitSwitchSource.Deactivated;
        public LimitSwitchNormal LIMIT_SWITCH_NORMAL = LimitSwitchNormal.Disabled;
        public double MAX_OUTPUT_VOLTAGE = 12;
        public double NOMINAL_VOLTAGE = 0;
        public double PEAK_VOLTAGE = 12;
        public boolean ENABLE_BRAKE = false;
        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int CURRENT_LIMIT = 0;
        //public double EXPIRATION_TIMEOUT_SECONDS = MotorSafety.DEFAULT_SAFETY_EXPIRATION;
        public int FORWARD_SOFT_LIMIT = 0;
        public boolean INVERTED = false;
        public int REVERSE_SOFT_LIMIT = 0;
        public int PID_TYPE = 0; // 0 = primary, 1 = cascaded
        
        public int TIMEOUT = 0; // in ms

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
        
        public int GYRO_STATUS_FRAME_PERIOD_MS = 100;
        public int MAG_STATUS_FRAME_PERIOD_MS = 100;
        public int ACCEL_STATUS_FRAME_PERIOD_MS = 100;

        
        public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public boolean ENABLE_VOLTAGE_COMPENSATION = false;
        public double VOLTAGE_COMPENSATION_RAMP_RATE = 0;
        public double VOLTAGE_RAMP_RATE = 0;
    }
	
	private static final Configuration defaultConfiguration = new Configuration();
	private static final Configuration slaveConfiguration = new Configuration();
	
	static {
        slaveConfiguration.CONTROL_FRAME_PERIOD_MS = 1000;
        slaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        slaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        slaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        slaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        slaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
    }
	
	/**
	 * Create a CANTalon with the default (out of the box) configuration.
	 * @param id
	 * @return create CANTalon
	 */
    public static TalonSRX createMasterTalon(int id) {
        return createTalon(id, defaultConfiguration);
    }
    
    /**
     * Creates a follower CANTalon with the default (out of the box) configuration
     * @param id
     * @param master_id
     * @return follower CANTalon
     */
    public static TalonSRX createFollowerTalon(int id, int master_id) {
        final TalonSRX talon = createTalon(id, slaveConfiguration);
        talon.set(ControlMode.Follower, master_id);
        return talon;
    }
    
    /**
     * Creates a new CANTalon
     * @param id
     * @param config
     * @return new CANTalon
     */
    public static TalonSRX createTalon(int id, Configuration config) {
        EfficientTalonSRX talon = new EfficientTalonSRX(id);
        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.setIntegralAccumulator(0, 0, config.TIMEOUT);
        talon.clearMotionProfileHasUnderrun(config.TIMEOUT);
        talon.clearMotionProfileTrajectories();
        talon.clearStickyFaults(config.TIMEOUT);
        talon.configForwardLimitSwitchSource(config.LIMIT_SWITCH_SOURCE, config.LIMIT_SWITCH_NORMAL, config.TIMEOUT);
        talon.configVoltageCompSaturation(config.MAX_OUTPUT_VOLTAGE, 0);
        talon.configNominalOutputForward(config.NOMINAL_VOLTAGE, config.TIMEOUT);
        talon.configNominalOutputReverse(-config.NOMINAL_VOLTAGE, config.TIMEOUT);
        talon.configPeakOutputForward(config.PEAK_VOLTAGE, config.TIMEOUT);
        talon.configPeakOutputReverse(-config.PEAK_VOLTAGE, config.TIMEOUT);
        talon.configReverseLimitSwitchSource(config.LIMIT_SWITCH_SOURCE, config.LIMIT_SWITCH_NORMAL, config.TIMEOUT);
        talon.setNeutralMode(NeutralMode.Coast);
        talon.enableCurrentLimit(config.ENABLE_CURRENT_LIMIT);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, config.TIMEOUT);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, config.TIMEOUT);
        talon.setInverted(config.INVERTED);
        talon.setSensorPhase(false);
        talon.configPeakCurrentLimit(config.CURRENT_LIMIT, config.TIMEOUT);
        talon.configContinuousCurrentLimit(config.CURRENT_LIMIT, config.TIMEOUT);
        talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, config.TIMEOUT);
        talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, config.TIMEOUT);
        talon.set(ControlMode.PercentOutput, 0);
        talon.selectProfileSlot(0, 0);
        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, config.TIMEOUT);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, config.TIMEOUT);
        talon.enableVoltageCompensation(config.ENABLE_VOLTAGE_COMPENSATION);
        talon.configOpenloopRamp(config.VOLTAGE_RAMP_RATE, config.TIMEOUT);
        talon.configClosedloopRamp(config.VOLTAGE_RAMP_RATE, config.TIMEOUT);
        //talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);
        //talon.setControlFramePeriod(ControlFrame.Control_4_Advanced, config.CONTROL_FRAME_PERIOD_MS);//TODO: Figure out how to change control frame
        
        talon.setStatusFramePeriod(StatusFrame.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, config.TIMEOUT);
        talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, config.TIMEOUT);
        talon.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, config.FEEDBACK_STATUS_FRAME_RATE_MS, config.TIMEOUT);
        talon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, config.FEEDBACK_STATUS_FRAME_RATE_MS, config.TIMEOUT);
        talon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, config.TIMEOUT);

        return talon;
    }
    
    public static PigeonIMU createPigeon(TalonSRX talon) {
    	return createPigeon(talon, defaultConfiguration);
    }
    
    public static PigeonIMU createPigeon(TalonSRX talon, Configuration config) {
    	PigeonIMU pigeon = new PigeonIMU(talon);
    	pigeon.setAccumZAngle(0, config.TIMEOUT);
    	pigeon.setControlFramePeriod(PigeonIMU_ControlFrame.Control_1, config.CONTROL_FRAME_PERIOD_MS);
    	pigeon.setFusedHeading(0, config.TIMEOUT);
    	pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, config.GYRO_STATUS_FRAME_PERIOD_MS, config.TIMEOUT);
    	pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, config.MAG_STATUS_FRAME_PERIOD_MS, config.TIMEOUT);
    	pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, config.ACCEL_STATUS_FRAME_PERIOD_MS, config.TIMEOUT);
    	pigeon.setYaw(0, config.TIMEOUT);
    	
    	return pigeon;
    }
    
    
}
