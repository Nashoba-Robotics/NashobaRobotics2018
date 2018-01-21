package edu.nr.robotics.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.sensorhistory.TalonEncoderClimber;
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
	private TalonEncoderClimber climberEncoder;
	
	public static final double ENC_TICKS_PER_INCH_CLIMBER = 0; //TODO: Find encoder ticks per inch for climber
	
	public static final Speed MAX_SPEED_CLIMBER = Speed.ZERO;//TODO: Find max speed of the climber
	
	/**
	 * The voltage ramp rate of the climber. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_CLIMBER = Time.ZERO; // TODO: Test for climber voltage ramp rate
	
	/**
	 * Current PID values for the climber
	 */
	public static final double F_CURRENT_CLIMBER = 1.00 * 1023 / MAX_SPEED_CLIMBER.get(Distance.Unit.MAGNETIC_ENCODER_TICK_CLIMBER, Time.Unit.HUNDRED_MILLISECOND);
	public static double P_CURRENT_CLIMBER = 0; // TODO: Find climber current PID values
	public static double I_CURRENT_CLIMBER = 0;
	public static double D_CURRENT_CLIMBER = 0;
	
	/**
	 * Position PID values for the climber
	 */
	public static double P_POS_CLIMBER = 0; //TODO: Find climber position PID values
	public static double I_POS_CLIMBER = 0;
	public static double D_POS_CLIMBER = 0; 
	
	/**
	 * The default current of the climber
	 */
	public static double DEFAULT_CLIMBER_CURRENT = 0; //TODO: Find default climber current
	
	/**
	 * The current values of the climber
	 */
	public static final int PEAK_CURRENT_CLIMBER = 0;// TODO: Find PEAK_CURRENT_CLIMBER
	public static final int PEAK_CURRENT_DURATION_CLIMBER = 0; // TODO: Find PEAK_CURRENT_DURATION_CLIMBER
	public static final int CONTINUOUS_CURRENT_LIMIT_CLIMBER = 0; // TODO: Find CONTINUOUS_CURRENT_LIMIT_CLIMBER
	
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
	 * The PID type of the climber. 0 = Primary, 1 = Cascade
	 */
	public static final int PID_TYPE = 0;
	
	/**
	 * The default timeout of the climber functions in ms
	 */
	public static final int DEFAULT_TIMEOUT = 0;
	
	/**
	 * The PID slot numbers
	 */
	public static final int CURRENT_SLOT = 0;
	public static final int POS_SLOT = 0;
	
	private double currentSetpoint = 0;
	
	private Climber() {
		if (EnabledSubsystems.CLIMBER_ENABLED) {
			climberTalon = CTRECreator.createMasterTalon(RobotMap.CLIMBER_TALON);
			
			if (EnabledSubsystems.CLIMBER_DUMB_ENABLED) {
				climberTalon.set(ControlMode.PercentOutput, 0);
			} else {
				climberTalon.set(ControlMode.Current, 0);
			}
				
			climberTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			climberTalon.config_kF(CURRENT_SLOT, F_CURRENT_CLIMBER, DEFAULT_TIMEOUT);
			climberTalon.config_kP(CURRENT_SLOT, P_CURRENT_CLIMBER, DEFAULT_TIMEOUT);
			climberTalon.config_kI(CURRENT_SLOT, I_CURRENT_CLIMBER, DEFAULT_TIMEOUT);
			climberTalon.config_kD(CURRENT_SLOT, D_CURRENT_CLIMBER, DEFAULT_TIMEOUT);
			
			climberTalon.config_kF(POS_SLOT, 0, DEFAULT_TIMEOUT);
			climberTalon.config_kP(POS_SLOT, P_POS_CLIMBER, DEFAULT_TIMEOUT);
			climberTalon.config_kI(POS_SLOT, I_POS_CLIMBER, DEFAULT_TIMEOUT);
			climberTalon.config_kD(POS_SLOT, D_POS_CLIMBER, DEFAULT_TIMEOUT);
			
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
	
			climberEncoder = new TalonEncoderClimber(climberTalon);
	
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
	 * @return The current velocity of the climber encoders
	 */
	public Speed getVelocity() {
		if (climberTalon != null)
			return new Speed(climberTalon.getSelectedSensorVelocity(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_CLIMBER,
				Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	/**
	 * Gets the historical velocity of the climber talon
	 * 
	 * @param timePassed
	 *            How long ago to look
	 * @return old velocity of the climber talon
	 */
	public Speed getHistoricalVelocity(Time timePassed) {
		if (climberTalon != null)
			return new Speed(climberEncoder.getVelocity(timePassed));
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
	
	/**
	 * @param current 
	 * 			The amps to set the climberTalon to
	 */
	public void setCurrent(double current) {
		if (climberTalon != null) {
			currentSetpoint = current;
			climberTalon.set(ControlMode.Current, current);
		}
	}
	
	/**
	 * Command that holds the climber in its current position
	 */
	public void holdPosition() {
		climberTalon.set(ControlMode.Position, climberTalon.getSelectedSensorPosition(PID_TYPE));
	}
	
	/**
	 * What is put on SmartDashboard when it's initialized
	 */
	public void smartDashboardInit() {
		if (EnabledSubsystems.CLIMBER_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Voltage Ramp Rate Climber Seconds: ",
					VOLTAGE_RAMP_RATE_CLIMBER.get(Time.Unit.SECOND));
			SmartDashboard.putNumber("P Current Climber: ", P_CURRENT_CLIMBER);
			SmartDashboard.putNumber("I Current Climber: ", I_CURRENT_CLIMBER);
			SmartDashboard.putNumber("D Current Climber: ", D_CURRENT_CLIMBER);
			SmartDashboard.putNumber("P Pos Climber: ", P_POS_CLIMBER);
			SmartDashboard.putNumber("I Pos Climber: ", I_POS_CLIMBER);
			SmartDashboard.putNumber("D Pos Climber: ", D_POS_CLIMBER);
			SmartDashboard.putNumber("Climber Set Current:", DEFAULT_CLIMBER_CURRENT);
		}
	}
	
	/**
	 * What is output or updated on SmartDashboard every time through the loop
	 */
		@Override
	public void smartDashboardInfo() {
			if (EnabledSubsystems.CLIMBER_SMARTDASHBOARD_BASIC_ENABLED) {
				SmartDashboard.putString("Climber Current vs Set Current: ", getCurrent() + " : " + currentSetpoint);
			}
			if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
				VOLTAGE_RAMP_RATE_CLIMBER = new Time(SmartDashboard.getNumber("Voltage Ramp Rate Climber Seconds: ", VOLTAGE_RAMP_RATE_CLIMBER.get(Time.Unit.SECOND)), Time.Unit.SECOND);
				P_CURRENT_CLIMBER = SmartDashboard.getNumber("P Current Climber: ", P_CURRENT_CLIMBER);
				I_CURRENT_CLIMBER = SmartDashboard.getNumber("I Current Climber: ", I_CURRENT_CLIMBER);
				D_CURRENT_CLIMBER = SmartDashboard.getNumber("D Current Climber: ", D_CURRENT_CLIMBER);
				P_POS_CLIMBER = SmartDashboard.getNumber("P Pos Climber: ", P_POS_CLIMBER);
				I_POS_CLIMBER = SmartDashboard.getNumber("I Pos Climber: ", I_POS_CLIMBER);
				D_POS_CLIMBER = SmartDashboard.getNumber("D Pos Climber: ", D_POS_CLIMBER);
				DEFAULT_CLIMBER_CURRENT = SmartDashboard.getNumber("Climber Set Current: ", DEFAULT_CLIMBER_CURRENT);
			}
		
	}
		
	@Override
	public void periodic() {

	}

	@Override
	public void disable() {
		climberTalon.set(ControlMode.PercentOutput, 0);
	}

}
