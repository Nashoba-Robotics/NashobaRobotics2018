package edu.nr.robotics.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.sensorhistory.TalonEncoder;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends NRSubsystem {
	
	private static Climber singleton;
	
	private TalonSRX climberTalon;
	
	public static final double ENC_TICKS_PER_INCH_CLIMBER = 0; //TODO: Find encoder ticks per inch for climber
	
	public static final Speed MAX_SPEED_CLIMBER = Speed.ZERO;//TODO: Find max speed of the climber
	
	public static final double CLIMB_PERCENT = 0.95;
	
	/**
	 * The voltage ramp rate of the climber. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_CLIMBER = new Time(0.05, Time.Unit.SECOND);
	
	/**
	 * The current values of the climber
	 */
	public static final int PEAK_CURRENT_CLIMBER = 80; 
	public static final int PEAK_CURRENT_DURATION_CLIMBER = 1000; //In milliseconds
	public static final int CONTINUOUS_CURRENT_LIMIT_CLIMBER = 40;
	
	/**
	 * The rate of velocity measurements on the climber encoder
	 */
	public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_CLIMBER = VelocityMeasPeriod.Period_10Ms;

	/**
	 * The number of measurements the climber encoder averages to return a
	 * value
	 */
	public static final double VELOCITY_MEASUREMENT_WINDOW_CLIMBER = 32;
	
	/**
	 * The 100% voltage that is used as a base calculation for all
	 * PercentOutputs
	 */
	public static final double VOLTAGE_COMPENSATION_LEVEL_CLIMBER = 12;
	
	/**
	 * The neutral mode of the climber (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE_CLIMBER = NeutralMode.Brake;
	
	/**
	 * Type of PID. 0 = primary. 1 = cascade
	 */
	public static final int PID_TYPE = 0;
	
	/**
	 * The default timeout of the climber functions in ms
	 */
	public static final int DEFAULT_TIMEOUT = 0;
		
	private Climber() {
		if (EnabledSubsystems.CLIMBER_ENABLED) {
			climberTalon = CTRECreator.createMasterTalon(RobotMap.CLIMBER_TALON);
			
			climberTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			
			climberTalon.setNeutralMode(NEUTRAL_MODE_CLIMBER);
			climberTalon.setInverted(false);
			climberTalon.setSensorPhase(false);
	
			climberTalon.enableVoltageCompensation(true);
			climberTalon.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_CLIMBER, DEFAULT_TIMEOUT);
	
			climberTalon.enableCurrentLimit(true);
			climberTalon.configPeakCurrentLimit(PEAK_CURRENT_CLIMBER, DEFAULT_TIMEOUT);
			climberTalon.configPeakCurrentDuration(PEAK_CURRENT_DURATION_CLIMBER, DEFAULT_TIMEOUT);
			climberTalon.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_CLIMBER, DEFAULT_TIMEOUT);
	
			climberTalon.configOpenloopRamp(VOLTAGE_RAMP_RATE_CLIMBER.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			climberTalon.configClosedloopRamp(VOLTAGE_RAMP_RATE_CLIMBER.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
		
		}

		smartDashboardInit();
	}
	
	
	public static Climber getInstance() {
		if (singleton == null)
			init();
		return singleton;
	}
	
	public synchronized static void init() {
		if (singleton == null) {
			singleton = new Climber();
		}
	}
	
	/**
	 * @return Current position of the climber encoders
	 */
	public Distance getPosition() {
		if (climberTalon != null) {
			return new Distance(climberTalon.getSelectedSensorPosition(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_CLIMBER);
		}
		return Distance.ZERO;
	}
	
	/**
	 * @return The current velocity of the climber encoders
	 */
	public Speed getVelocity() {
		if (climberTalon != null)
			return new Speed(climberTalon.getSelectedSensorVelocity(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_CLIMBER,
				Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	/**
	 * @return The current of the climber talon in amps
	 */
	public double getCurrent() {
		if (climberTalon != null)
			return climberTalon.getOutputCurrent();
		return 0;
	}
	
	public void setMotorSpeedPercent(double percent) {
		if (climberTalon != null) {
			climberTalon.set(ControlMode.PercentOutput, percent);
		}
	}
	
	public void setCoastMode(boolean bool) {
		if (climberTalon != null) {
			if (bool) {
				climberTalon.setNeutralMode(NeutralMode.Coast);	
			} else {
				climberTalon.setNeutralMode(NeutralMode.Brake);
			}
		}
	}
	
	/**
	 * What is put on SmartDashboard when it's initialized
	 */
	public void smartDashboardInit() {
		if (EnabledSubsystems.CLIMBER_SMARTDASHBOARD_DEBUG_ENABLED) {
			
		}
	}
	
	/**
	 * What is output or updated on SmartDashboard every time through the loop
	 */
		@Override
	public void smartDashboardInfo() {
			if (EnabledSubsystems.CLIMBER_SMARTDASHBOARD_BASIC_ENABLED) {
				SmartDashboard.putNumber("Climber Current", getCurrent());
			}
			if (EnabledSubsystems.CLIMBER_SMARTDASHBOARD_DEBUG_ENABLED) {
				SmartDashboard.putNumber("Climber Encoder Ticks: ", climberTalon.getSelectedSensorPosition(PID_TYPE));
			}
		
	}
		
	@Override
	public void periodic() {

	}

	@Override
	public void disable() {
		if (EnabledSubsystems.CLIMBER_ENABLED) {
			climberTalon.set(ControlMode.PercentOutput, 0);	
		}
	}

}
