package edu.nr.robotics.subsystems.elevator;

import com.ctre.CANTalon.VelocityMeasurementPeriod;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;

public class Elevator extends NRSubsystem{

	private static Elevator elevator;
	
	/**
	 * The encoder ticks per inch moved on the elevator
	 */
	public static final double ENC_TICK_PER_INCH_ELEVATOR = 0; //TODO: Find ENC_TICK_PER_INCH_ELEVATOR
	
	/**
	 * The max speed of the elevator
	 */
	public static final Speed MAX_SPEED_ELEVATOR = Speed.ZERO; //TODO: Find MAX_SPEED_ELEVATOR
	
	/**
	 * The max acceleration of the elevator
	 */
	public static final Acceleration MAX_ACCEL_ELEVATOR = Acceleration.ZERO; //TODO: Find MAX_ACCEL_ELEVATOR
	
	/**
	 * The minimum voltage needed to move the elevator
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR = 0; //TODO: Find Elevator voltage velocity curve
	
	/**
	 * The slope of voltage over velocity in feet per second
	 */
	public static final double VOLTAGE_VELOCITY_SLOPE_ELEVATOR = 0;
	
	/**
	 * The voltage ramp rate of the elevator.
	 * Voltage ramp rate is time it takes to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_ELEVATOR = Time.ZERO; //TODO: Test for elevator voltage ramp rate
	
	/**
	 * MotionMagic PID values for the elevator
	 */
	public static double P_POS_ELEVATOR = 0; //TODO: Find elevator MagicMotion PID values
	public static double I_POS_ELEVATOR = 0;
	public static double D_POS_ELEVATOR = 0;
	
	/**
	 * Velocity PID values for the elevator
	 */
	public static double P_VEL_ELEVATOR = 0; //TODO: Find elevator velocity PID values
	public static double I_VEL_ELEVATOR = 0;
	public static double D_VEL_ELEVATOR = 0;
	
	/**
	 * The default profiling velocity percent of the elevator
	 */
	public static double PROFILE_VEL_PERCENT_ELEVATOR = 0; //TODO: Decide on PROFILE_VEL_PERCENT_ELEVATOR
	
	/**
	 * The default profiling acceleration of the elevator
	 */
	public static double PROFILE_ACCEL_PERCENT_ELEVATOR = 0; //TODO: Decide on PROFILE_ACCEL_PERCENT_ELEVATOR
	
	/**
	 * The distance from the end of the elevator profile at which the stopping algorithm
	 * 		starts
	 */
	public static final double PROFILE_END_POS_THRESHOLD_ELEVATOR = 0; //TODO: Decide on PROFILE_END_POS_THRESHOLD_ELEVATOR
	
	/**
	 * The change in position elevator within for PROFILE_TIME_POS_THRESHOLD_ELEVATOR before stopping profile
	 */
	public static final double PROFILE_DELTA_POS_THRESHOLD_ELEVATOR = 0; //TODO: Decide on PROFILE_DELTA_POS_THRESHOLD_ELEVATOR
	public static final double PROFILE_DELTA_TIME_THRESHOLD_ELEVATOR = 0; //TODO: Decide on PROFILE_DELTA_TIME_THRESHOLD_ELEVATOR
	
	/**
	 * The current values of the elevator
	 */
	public static final double PEAK_CURRENT_ELEVATOR = 0;//TODO: Find PEAK_CURRENT_ELEVATOR
	public static final double PEAK_CURRENT_DURATION_ELEVATOR = 0; //TODO: Find PEAK_CURRENT_DURATION_ELEVATOR
	public static final double CONTINUOUS_CURRENT_LIMIT_ELEVATOR = 0; //TODO: Find CONTINUOUS_CURRENT_LIMIT_ELEVATOR
	
	/**
	 * The rate of velocity measurements on the elevator encoder
	 */
	public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_ELEVATOR = VelocityMeasPeriod.Period_10Ms;
	
	/**
	 * The number of measurements the elevator encoder averages to return a value
	 */
	public static final double VELOCITY_MEASUREMENT_WINDOW_ELEVATOR = 32;
	
	/**
	 * The 100% voltage that is used as a base calculation for all PercentOutputs
	 */
	public static final double VOLTAGE_COMPENSATION_LEVEL_ELEVATOR = 12;
	
	/**
	 * The neutral mode of the elevator (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE_ELEVATOR = NeutralMode.Brake;
	
	/**
	 * The PID type of the elevator. 0 = Primary, 1 = Cascade
	 */
	public static final double PID_TYPE = 0;
	
	/**
	 * The default timeout of the elevator functions in ms
	 */
	public static final double DEFAULT_TIMEOUT = 0;
	
	/**
	 * The PID slot numbers 
	 */
	public static final double VEL_SLOT = 0;
	public static final double MOTION_MAGIC_SLOT = 1;
	
	/**
	 * The positions of the elevator at each limit switch and at the default extend height
	 */
	public static final Distance TOP_POSITION_ELEVATOR = Distance.ZERO; //TODO: Find TOP_POSITION_ELEVATOR
	public static final Distance AUTO_HEIGHT_ELEVATOR = Distance.ZERO; //TODO: Find AUTO_HEIGHT_ELEVATOR
	public static final Distance BOTTOM_HEIGHT_ELEVATOR = Distance.ZERO; //TODO: Find BOTTOM_HEIGHT_ELEVATOR
	
	@Override
	public void smartDashboardInfo() {
		
	}

	@Override
	public void disable() {
		
	}

}
