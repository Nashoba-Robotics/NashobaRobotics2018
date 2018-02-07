package edu.nr.robotics.subsystems.elevator;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.sensorhistory.TalonEncoder;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends NRSubsystem {

	private static Elevator singleton;

	private TalonSRX elevTalon, elevTalonFollow;
	private TalonEncoder elevEncoder;

	/**
	 * The encoder ticks per inch moved on the elevator
	 */
	public static final double ENC_TICK_PER_INCH_CARRIAGE = 0; // TODO: Find ENC_TICK_PER_INCH_CARRIAGE

	/**
	 * The max speed of the elevator
	 */
	public static final Speed MAX_SPEED_ELEVATOR = Speed.ZERO; // TODO: Find MAX_SPEED_ELEVATOR

	/**
	 * The max acceleration of the elevator
	 */
	public static final Acceleration MAX_ACCEL_ELEVATOR = Acceleration.ZERO; // TODO: Find MAX_ACCEL_ELEVATOR

	/**
	 * The minimum voltage needed to move the elevator
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR = 0; // TODO: Find Elevator voltage velocity curve

	/**
	 * The slope of voltage over velocity in feet per second
	 */
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR = 0;

	/**
	 * The voltage ramp rate of the elevator. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_ELEVATOR = Time.ZERO; // TODO: Test for elevator voltage ramp rate

	/**
	 * MotionMagic PID values for the elevator
	 */
	public static double F_POS_ELEVATOR = 1.00 * 1023 / MAX_SPEED_ELEVATOR.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND);
	public static double P_POS_ELEVATOR = 0; // TODO: Find elevator MagicMotion PID values
	public static double I_POS_ELEVATOR = 0;
	public static double D_POS_ELEVATOR = 0;

	/**
	 * Velocity PID values for the elevator
	 */
	public static double P_VEL_ELEVATOR = 0; // TODO: Find elevator velocity PID values
	public static double I_VEL_ELEVATOR = 0;
	public static double D_VEL_ELEVATOR = 0;

	/**
	 * The default profiling velocity percent of the elevator
	 */
	public static double PROFILE_VEL_PERCENT_ELEVATOR = 0; // TODO: Decide on PROFILE_VEL_PERCENT_ELEVATOR

	/**
	 * The default profiling acceleration of the elevator
	 */
	public static double PROFILE_ACCEL_PERCENT_ELEVATOR = 0; // TODO: Decide on PROFILE_ACCEL_PERCENT_ELEVATOR

	/**
	 * The distance from the end of the elevator profile at which the stopping
	 * algorithm starts
	 */
	public static final Distance PROFILE_END_POS_THRESHOLD_ELEVATOR = Distance.ZERO; // TODO: Decide on PROFILE_END_POS_THRESHOLD_ELEVATOR

	/**
	 * The change in position elevator within for
	 * PROFILE_TIME_POS_THRESHOLD_ELEVATOR before stopping profile
	 */
	public static final Distance PROFILE_DELTA_POS_THRESHOLD_ELEVATOR = Distance.ZERO; // TODO: Decide on PROFILE_DELTA_POS_THRESHOLD_ELEVATOR
	public static final Time PROFILE_DELTA_TIME_THRESHOLD_ELEVATOR = Time.ZERO; // TODO: Decide on PROFILE_DELTA_TIME_THRESHOLD_ELEVATOR

	/**
	 * The current values of the elevator
	 */
	public static final int PEAK_CURRENT_ELEVATOR = 0;// TODO: Find PEAK_CURRENT_ELEVATOR
	public static final int PEAK_CURRENT_DURATION_ELEVATOR = 0; // TODO: Find PEAK_CURRENT_DURATION_ELEVATOR
	public static final int CONTINUOUS_CURRENT_LIMIT_ELEVATOR = 0; // TODO: Find CONTINUOUS_CURRENT_LIMIT_ELEVATOR

	/**
	 * The rate of velocity measurements on the elevator encoder
	 */
	public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_ELEVATOR = VelocityMeasPeriod.Period_10Ms;

	/**
	 * The number of measurements the elevator encoder averages to return a
	 * value
	 */
	public static final double VELOCITY_MEASUREMENT_WINDOW_ELEVATOR = 32;

	/**
	 * The 100% voltage that is used as a base calculation for all
	 * PercentOutputs
	 */
	public static final double VOLTAGE_COMPENSATION_LEVEL_ELEVATOR = 12;

	/**
	 * The neutral mode of the elevator (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE_ELEVATOR = NeutralMode.Brake;

	/**
	 * The PID type of the elevator. 0 = Primary, 1 = Cascade
	 */
	public static final int PID_TYPE = 0;

	/**
	 * The default timeout of the elevator functions in ms
	 */
	public static final int DEFAULT_TIMEOUT = 0;

	/**
	 * The PID slot numbers
	 */
	public static final int VEL_SLOT = 0;
	public static final int MOTION_MAGIC_SLOT = 1;
	
	/**
	 * The max speed ratio of the carriage to the hook
	 */
	public static final double HOOK_TO_CARRIAGE_RATIO = 0; //TODO: Find  Carriage to hook ratio

	/**
	 * The positions of the elevator at each limit switch and at the default
	 * extend height
	 */
	public static final Distance TOP_HEIGHT_ELEVATOR = Distance.ZERO; // TODO: Find TOP_POSITION_ELEVATOR
	public static final Distance CLIMB_HEIGHT_ELEVATOR = Distance.ZERO; //TODO: Find CLIMB_HEIGHT_ELEVATOR
	public static final Distance SCALE_HEIGHT_ELEVATOR = Distance.ZERO; // TODO: Find AUTO_HEIGHT_ELEVATOR
	public static final Distance SWITCH_HEIGHT_ELEVATOR = Distance.ZERO; //TODO: Find SCORE_LOW_HEIGHT_ELEVATOR
	public static final Distance BOTTOM_HEIGHT_ELEVATOR = Distance.ZERO;

	private Speed velSetpoint = Speed.ZERO;
	private Distance posSetpoint = Distance.ZERO;

	public static Distance profileDeltaPos = Distance.ZERO;

	private Elevator() {

		if (EnabledSubsystems.ELEVATOR_ENABLED) {
			elevTalon = CTRECreator.createMasterTalon(RobotMap.ELEVATOR_TALON);
			elevTalonFollow = CTRECreator.createFollowerTalon(RobotMap.ELEVATOR_TALON_FOLLOW, elevTalon.getDeviceID());
	
			if (EnabledSubsystems.ELEVATOR_DUMB_ENABLED) {
				elevTalon.set(ControlMode.PercentOutput, 0);
			} else {
				elevTalon.set(ControlMode.Velocity, 0);
			}
	
			elevTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			elevTalon.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
			elevTalon.config_kP(VEL_SLOT, P_VEL_ELEVATOR, DEFAULT_TIMEOUT);
			elevTalon.config_kI(VEL_SLOT, I_VEL_ELEVATOR, DEFAULT_TIMEOUT);
			elevTalon.config_kD(VEL_SLOT, D_VEL_ELEVATOR, DEFAULT_TIMEOUT);
			elevTalon.config_kF(MOTION_MAGIC_SLOT, 0, DEFAULT_TIMEOUT);
			elevTalon.config_kP(MOTION_MAGIC_SLOT, P_POS_ELEVATOR, DEFAULT_TIMEOUT);
			elevTalon.config_kI(MOTION_MAGIC_SLOT, I_POS_ELEVATOR, DEFAULT_TIMEOUT);
			elevTalon.config_kD(MOTION_MAGIC_SLOT, D_POS_ELEVATOR, DEFAULT_TIMEOUT);
			elevTalon.setNeutralMode(NEUTRAL_MODE_ELEVATOR);
			elevTalon.setInverted(false);
			elevTalon.setSensorPhase(false);
	
			elevTalon.enableVoltageCompensation(true);
			elevTalon.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_ELEVATOR, DEFAULT_TIMEOUT);
	
			elevTalon.enableCurrentLimit(true);
			elevTalon.configPeakCurrentLimit(PEAK_CURRENT_ELEVATOR, DEFAULT_TIMEOUT);
			elevTalon.configPeakCurrentDuration(PEAK_CURRENT_DURATION_ELEVATOR, DEFAULT_TIMEOUT);
			elevTalon.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_ELEVATOR, DEFAULT_TIMEOUT);
	
			elevTalon.configClosedloopRamp(VOLTAGE_RAMP_RATE_ELEVATOR.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			elevTalon.configOpenloopRamp(VOLTAGE_RAMP_RATE_ELEVATOR.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
	
			elevTalon.configMotionCruiseVelocity((int) MAX_SPEED_ELEVATOR.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND),
							DEFAULT_TIMEOUT);
			elevTalon.configMotionAcceleration((int) MAX_ACCEL_ELEVATOR.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
			
			elevTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, DEFAULT_TIMEOUT);
			elevTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, DEFAULT_TIMEOUT);
	
			elevEncoder = new TalonEncoder(elevTalon, Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV);
	
			elevTalonFollow.setNeutralMode(NEUTRAL_MODE_ELEVATOR);
		}

		smartDashboardInit();
	}

	public static Elevator getInstance() {
		if (singleton == null)
			init();
		return singleton;
	}

	public synchronized static void init() {
		if (singleton == null) {
			singleton = new Elevator();
			singleton.setJoystickCommand(new ElevatorJoystickCommand());
		}
	}

	/**
	 * @return The current position of the elevator encoders
	 */
	public Distance getPosition() {
		if (elevTalon != null)
			return new Distance(elevTalon.getSelectedSensorPosition(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV);
		return Distance.ZERO;
	}

	/**
	 * Gets the historical position of the elevator talon
	 * 
	 * @param timePassed
	 *            How long ago to look
	 * @return old position of the elevator talon
	 */
	public Distance getHistoricalPosition(Time timePassed) {
		if (elevTalon != null)
			return elevEncoder.getPosition(timePassed);
		return Distance.ZERO;
	}

	/**
	 * @return The current velocity of the elevator encoders
	 */
	public Speed getVelocity() {
		if (elevTalon != null)
			return new Speed(elevTalon.getSelectedSensorVelocity(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
				Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}

	/**
	 * Gets the historical velocity of the elevator talon
	 * 
	 * @param timePassed
	 *            How long ago to look
	 * @return old velocity of the elevator talon
	 */
	public Speed getHistoricalVelocity(Time timePassed) {
		if (elevTalon != null)
			return new Speed(elevEncoder.getVelocity(timePassed));
		return Speed.ZERO;
	}

	/**
	 * @return The current of the elevator talon
	 */
	public double getCurrent() {
		if (elevTalon != null)
			return elevTalon.getOutputCurrent();
		return 0;
	}

	/**
	 * @param position
	 *            the absolute position the elevator talon should go to (0+ from
	 *            BOTTOM_HEIGHT up)
	 */
	public void setPosition(Distance position) {
		if (elevTalon != null) {
			posSetpoint = position;
			velSetpoint = Speed.ZERO;
			elevTalon.configMotionCruiseVelocity((int) MAX_SPEED_ELEVATOR.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND),
							DEFAULT_TIMEOUT);
			elevTalon.configMotionAcceleration((int) MAX_ACCEL_ELEVATOR.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
	
			elevTalon.set(ControlMode.MotionMagic, position.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV));
		}
	}

	/**
	 * @param percent velocity
	 */
	public void setMotorSpeedPercent(double percent) {
		setMotorSpeed(MAX_SPEED_ELEVATOR.mul(percent));
	}

	/**
	 * @param speed to set elevator carriage in
	 */
	public void setMotorSpeed(Speed speed) {

		if (elevTalon != null) {
			velSetpoint = speed;
			posSetpoint = Distance.ZERO;
			elevTalon.config_kF(VEL_SLOT,
					((VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR * velSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
							+ MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR) * 1023.0)
							/ velSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
									Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
	
			if (elevTalon.getControlMode() == ControlMode.PercentOutput) {
				elevTalon.set(elevTalon.getControlMode(), velSetpoint.div(MAX_SPEED_ELEVATOR));
			} else {
				elevTalon.set(elevTalon.getControlMode(),
						velSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND));
			}
		}
	}

	/**
	 * Sets the time limit going from 0V to 12V in seconds
	 * 
	 * @param time
	 *            going from 0V to 12V
	 */
	public void setVoltageRamp(Time time) {
		if (elevTalon != null) {
			elevTalon.configOpenloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			elevTalon.configClosedloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
		}
	}

	/**
	 * What is put on SmartDashboard when it's initialized
	 */
	public void smartDashboardInit() {
		if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Elevator Profile Delta Inches: ", 0);
			SmartDashboard.putNumber("Voltage Ramp Rate Elevator Seconds: ",
					VOLTAGE_RAMP_RATE_ELEVATOR.get(Time.Unit.SECOND));
			SmartDashboard.putNumber("P Pos Elevator: ", P_POS_ELEVATOR);
			SmartDashboard.putNumber("I Pos Elevator: ", I_POS_ELEVATOR);
			SmartDashboard.putNumber("D Pos Elevator: ", D_POS_ELEVATOR);
			SmartDashboard.putNumber("P Vel Elevator: ", P_VEL_ELEVATOR);
			SmartDashboard.putNumber("I Vel Elevator: ", I_VEL_ELEVATOR);
			SmartDashboard.putNumber("D Vel Elevator: ", D_VEL_ELEVATOR);
			SmartDashboard.putNumber("Profile Vel Percent Elevator: ", PROFILE_VEL_PERCENT_ELEVATOR);
			SmartDashboard.putNumber("Profile Accel Percent Elevator: ", PROFILE_ACCEL_PERCENT_ELEVATOR);
		}
	}

	/**
	 * What is output or updated on SmartDashboard every time through the loop
	 */
	@Override
	public void smartDashboardInfo() {
		if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_BASIC_ENABLED) {
			SmartDashboard.putNumber("Elevator Current: ", getCurrent());
			SmartDashboard.putString("Elevator Velocity vs. Set Velocity: ",
					getVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : "
							+ velSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
			SmartDashboard.putString("Elevator Position vs. Set Position: ",
					getPosition().get(Distance.Unit.INCH) + " : " + posSetpoint.get(Distance.Unit.INCH));
		}
		if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
			profileDeltaPos = new Distance(SmartDashboard.getNumber("Elevator Profile Delta Inches: ", 0), Distance.Unit.INCH);
			P_POS_ELEVATOR = SmartDashboard.getNumber("P Pos Elevator: ", P_POS_ELEVATOR);
			I_POS_ELEVATOR = SmartDashboard.getNumber("I Pos Elevator: ", I_POS_ELEVATOR);
			D_POS_ELEVATOR = SmartDashboard.getNumber("D Pos Elevator: ", D_POS_ELEVATOR);
			P_VEL_ELEVATOR = SmartDashboard.getNumber("P Vel Elevator: ", P_VEL_ELEVATOR);
			I_VEL_ELEVATOR = SmartDashboard.getNumber("I Vel Elevator: ", I_VEL_ELEVATOR);
			D_VEL_ELEVATOR = SmartDashboard.getNumber("D Vel Elevator: ", D_VEL_ELEVATOR);
			PROFILE_VEL_PERCENT_ELEVATOR = SmartDashboard.getNumber("Profile Vel Percent Elevator: ",
					PROFILE_VEL_PERCENT_ELEVATOR);
			PROFILE_ACCEL_PERCENT_ELEVATOR = SmartDashboard.getNumber("Profile Accel Percent Elevator: ",
					PROFILE_ACCEL_PERCENT_ELEVATOR);
			
			SmartDashboard.putNumber("Elevator Encoder Ticks: ", elevTalon.getSelectedSensorPosition(PID_TYPE));
		}
	}
	
	public boolean isRevLimitSwitchClosed() {
		return elevTalon.getSensorCollection().isRevLimitSwitchClosed();
	}

	@Override
	public void periodic() {
		if (EnabledSubsystems.ELEVATOR_ENABLED) {
			if (elevTalon.getSensorCollection().isFwdLimitSwitchClosed()) {
				elevTalon.getSensorCollection().setQuadraturePosition((int) TOP_HEIGHT_ELEVATOR.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV), DEFAULT_TIMEOUT);
			}
			if (elevTalon.getSensorCollection().isRevLimitSwitchClosed()) {
				elevTalon.getSensorCollection().setQuadraturePosition((int) BOTTOM_HEIGHT_ELEVATOR.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV), DEFAULT_TIMEOUT);
			}
		}
	}

	@Override
	public void disable() {
		setMotorSpeedPercent(0);
	}
}
