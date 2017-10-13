package edu.nr.lib.talons;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.MotorSafety;

/**
 * Adapted from 254's CANTalonFactory
 * 
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */

public class TalonCreator {

	public static class Configuration {
        public boolean LIMIT_SWITCH_NORMALLY_OPEN = true;
        public double MAX_OUTPUT_VOLTAGE = 12;
        public double NOMINAL_VOLTAGE = 0;
        public double PEAK_VOLTAGE = 12;
        public boolean ENABLE_BRAKE = false;
        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int CURRENT_LIMIT = 0;
        public double EXPIRATION_TIMEOUT_SECONDS = MotorSafety.DEFAULT_SAFETY_EXPIRATION;
        public double FORWARD_SOFT_LIMIT = 0;
        public boolean INVERTED = false;
        public double NOMINAL_CLOSED_LOOP_VOLTAGE = 12;
        public double REVERSE_SOFT_LIMIT = 0;
        public boolean SAFETY_ENABLED = false;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 100;

        public CANTalon.VelocityMeasurementPeriod VELOCITY_MEASUREMENT_PERIOD = CANTalon.VelocityMeasurementPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

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
        slaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }
	
	/**
	 * Create a CANTalon with the default (out of the box) configuration.
	 * @param id
	 * @return create CANTalon
	 */
    public static CANTalon createMasterTalon(int id) {
        return createTalon(id, defaultConfiguration);
    }
    
    /**
     * Creates a follower CANTalon with the default (out of the box) configuration
     * @param id
     * @param master_id
     * @return follower CANTalon
     */
    public static CANTalon createFollowerTalon(int id, int master_id) {
        final CANTalon talon = createTalon(id, slaveConfiguration);
        talon.changeControlMode(TalonControlMode.Follower);
        talon.set(master_id);
        return talon;
    }
    
    /**
     * Creates a new CANTalon
     * @param id
     * @param config
     * @return new CANTalon
     */
    public static CANTalon createTalon(int id, Configuration config) {
        EfficientCANTalon talon = new EfficientCANTalon(id, config.CONTROL_FRAME_PERIOD_MS);
        talon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.clearIAccum();
        talon.ClearIaccum();
        talon.clearMotionProfileHasUnderrun();
        talon.clearMotionProfileTrajectories();
        talon.clearStickyFaults();
        talon.ConfigFwdLimitSwitchNormallyOpen(config.LIMIT_SWITCH_NORMALLY_OPEN);
        talon.configMaxOutputVoltage(config.MAX_OUTPUT_VOLTAGE);
        talon.configNominalOutputVoltage(config.NOMINAL_VOLTAGE, -config.NOMINAL_VOLTAGE);
        talon.configPeakOutputVoltage(config.PEAK_VOLTAGE, -config.PEAK_VOLTAGE);
        talon.ConfigRevLimitSwitchNormallyOpen(config.LIMIT_SWITCH_NORMALLY_OPEN);
        talon.enableBrakeMode(config.ENABLE_BRAKE);
        talon.EnableCurrentLimit(config.ENABLE_CURRENT_LIMIT);
        talon.enableForwardSoftLimit(config.ENABLE_SOFT_LIMIT);
        talon.enableLimitSwitch(config.ENABLE_LIMIT_SWITCH, config.ENABLE_LIMIT_SWITCH);
        talon.enableReverseSoftLimit(config.ENABLE_SOFT_LIMIT);
        talon.enableZeroSensorPositionOnForwardLimit(false);
        talon.enableZeroSensorPositionOnIndex(false, false);
        talon.enableZeroSensorPositionOnReverseLimit(false);
        talon.reverseOutput(false);
        talon.reverseSensor(false);
        talon.setAnalogPosition(0);
        talon.setCurrentLimit(config.CURRENT_LIMIT);
        talon.setExpiration(config.EXPIRATION_TIMEOUT_SECONDS);
        talon.setForwardSoftLimit(config.FORWARD_SOFT_LIMIT);
        talon.setInverted(config.INVERTED);
        talon.setNominalClosedLoopVoltage(config.NOMINAL_CLOSED_LOOP_VOLTAGE);
        talon.setPosition(0);
        talon.setProfile(0);
        talon.setPulseWidthPosition(0);
        talon.setReverseSoftLimit(config.REVERSE_SOFT_LIMIT);
        talon.setSafetyEnabled(config.SAFETY_ENABLED);
        talon.SetVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD);
        talon.SetVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW);
        talon.setVoltageCompensationRampRate(config.VOLTAGE_COMPENSATION_RAMP_RATE);
        talon.setVoltageRampRate(config.VOLTAGE_RAMP_RATE);

        talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.General, config.GENERAL_STATUS_FRAME_RATE_MS);
        talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, config.FEEDBACK_STATUS_FRAME_RATE_MS);
        talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.QuadEncoder, config.QUAD_ENCODER_STATUS_FRAME_RATE_MS);
        talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.AnalogTempVbat,
                config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS);
        talon.setStatusFrameRateMs(CANTalon.StatusFrameRate.PulseWidth, config.PULSE_WIDTH_STATUS_FRAME_RATE_MS);

        return talon;
    }
    
    
}
