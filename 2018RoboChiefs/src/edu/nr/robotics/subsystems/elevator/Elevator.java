package edu.nr.robotics.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.commandbased.CancelCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerBasic;
import edu.nr.lib.motionprofiling.OneDimensionalTrajectoryRamped;
import edu.nr.lib.motionprofiling.OneDimensionalTrajectorySimple;
import edu.nr.lib.sensorhistory.TalonEncoder;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends NRSubsystem implements PIDOutput, PIDSource {

	private static Elevator singleton;

	private TalonSRX elevTalon, elevTalonFollow;
	private TalonEncoder elevEncoder;

	/**
	 * The encoder ticks per inch moved on the elevator
	 */
	public static final double ENC_TICK_PER_INCH_CARRIAGE = 56535.0 / 107.0;
	
	/**
	 * The max speed of the elevator
	 */
	public static final Speed MAX_SPEED_ELEVATOR_UP = new Speed(12.77, Distance.Unit.FOOT, Time.Unit.SECOND);
	public static final Speed MAX_SPEED_ELEVATOR_DOWN = Speed.ZERO; //TODO: Find MAX_SPEED_ELEVATOR_DOWN

	/**
	 * The max acceleration of the elevator
	 */
	public static final Acceleration MAX_ACCEL_ELEVATOR_UP = new Acceleration(28, Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND);
	public static final Acceleration MAX_ACCEL_ELEVATOR_DOWN = Acceleration.ZERO; //TODO: Find MAX_ACCEL_ELEVATOR_DOWN

	public static final double REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP = 0.27;
	public static final double REAL_MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN = 0.27;//0.20
	
	/**
	 * The minimum voltage needed to move the elevator
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP = 0.304;
	public static final double MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN = 0; //TODO: Find MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN

	/**
	 * The slope of voltage over velocity in feet per second
	 */
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_UP = 0.0545;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_DOWN = 0; //TODO: Find VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_DOWN
	
	/**
	 * The voltage ramp rate of the elevator. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_ELEVATOR = Time.ZERO; // TODO: Test for elevator voltage ramp rate

	/**
	 * The default profiling velocity percent of the elevator
	 */
	public static double PROFILE_VEL_PERCENT_ELEVATOR = 0.8;

	/**
	 * The default profiling acceleration of the elevator
	 */
	public static double PROFILE_ACCEL_PERCENT_ELEVATOR = 0.9;

	/**
	 * MotionMagic PID values for the elevator
	 */
	public static double F_POS_ELEVATOR_UP = 0;
	
	public static double P_POS_ELEVATOR_UP = 0; // TODO: Find elevator MagicMotion PID values
	public static double I_POS_ELEVATOR_UP = 0;
	public static double D_POS_ELEVATOR_UP = 0;
	
	public static double F_POS_ELEVATOR_DOWN = ((VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_DOWN * MAX_SPEED_ELEVATOR_DOWN.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
			+ MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN) * 1023.0)
			/ MAX_SPEED_ELEVATOR_DOWN.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
					Time.Unit.HUNDRED_MILLISECOND);
	
	public static double P_POS_ELEVATOR_DOWN = 0; // TODO: Find elevator MagicMotion PID values
	public static double I_POS_ELEVATOR_DOWN = 0;
	public static double D_POS_ELEVATOR_DOWN = 0;
	
	/**
	 * Velocity PID values for the elevator
	 */
	public static double P_VEL_ELEVATOR_UP = 0.08;
	public static double I_VEL_ELEVATOR_UP = 0;
	public static double D_VEL_ELEVATOR_UP = 0.8;
	
	public static double P_VEL_ELEVATOR_DOWN = 0; // TODO: Find elevator velocity PID values for down
	public static double I_VEL_ELEVATOR_DOWN = 0;
	public static double D_VEL_ELEVATOR_DOWN = 0;
	
	/**
	 * The distance from the end of the elevator profile at which the stopping
	 * algorithm starts
	 */
	public static final Distance PROFILE_END_POS_THRESHOLD_ELEVATOR = new Distance(10, Distance.Unit.INCH); // TODO: Decide on PROFILE_END_POS_THRESHOLD_ELEVATOR

	/**
	 * The change in position elevator within for
	 * PROFILE_TIME_POS_THRESHOLD_ELEVATOR before stopping profile
	 */
	public static final Distance PROFILE_DELTA_POS_THRESHOLD_ELEVATOR = new Distance(2, Distance.Unit.INCH); // TODO: Decide on PROFILE_DELTA_POS_THRESHOLD_ELEVATOR
	public static final Time PROFILE_DELTA_TIME_THRESHOLD_ELEVATOR = new Time(0.1, Time.Unit.SECOND); // TODO: Decide on PROFILE_DELTA_TIME_THRESHOLD_ELEVATOR

	/**
	 * Time multiplier for the ramp of the ramped trajectory
	 */
	public static final double RAMPED_PROFILE_TIME_MULT_ELEVATOR = 1000;
	
	/**
	 * The current values of the elevator
	 */
	public static final int PEAK_CURRENT_ELEVATOR = 80;
	public static final int PEAK_CURRENT_DURATION_ELEVATOR = 1000;
	public static final int CONTINUOUS_CURRENT_LIMIT_ELEVATOR = 40;

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
	public static final int VEL_UP_SLOT = 0;
	public static final int MOTION_MAGIC_UP_SLOT = 1;
	public static final int VEL_DOWN_SLOT = 2;
	public static final int MOTION_MAGIC_DOWN_SLOT = 3;
	
	public static final double kV_UP = 1 / MAX_SPEED_ELEVATOR_UP.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND);
	public static double kA_UP = 0;
	public static double kP_UP = 0;
	public static double kD_UP = 0;
	
	public static final double kV_DOWN = 1 / MAX_SPEED_ELEVATOR_DOWN.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND);
	public static double kA_DOWN = 0;
	public static double kP_DOWN = 0;
	public static double kD_DOWN = 0;
	
	/**
	 * The max speed ratio of the carriage to the hook
	 */
	public static final double HOOK_TO_CARRIAGE_RATIO = 0; //TODO: Find  Carriage to hook ratio

	/**
	 * The positions of the elevator at each limit switch and at the default
	 * extend height
	 */
	public static final Distance TOP_HEIGHT_ELEVATOR = new Distance(56535.0, Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV); // TODO: Find TOP_POSITION_ELEVATOR
	public static final Distance CLIMB_HEIGHT_ELEVATOR = Distance.ZERO; //TODO: Find CLIMB_HEIGHT_ELEVATOR
	public static final Distance SCALE_HEIGHT_ELEVATOR = Distance.ZERO; // TODO: Find AUTO_HEIGHT_ELEVATOR
	public static final Distance SWITCH_HEIGHT_ELEVATOR = new Distance(5, Distance.Unit.INCH); //TODO: Find SCORE_LOW_HEIGHT_ELEVATOR
	public static final Distance BOTTOM_HEIGHT_ELEVATOR = Distance.ZERO;
	
	private Speed velSetpoint = Speed.ZERO;
	private Distance posSetpoint = Distance.ZERO;

	public static Distance profilePos = Distance.ZERO;
	
	public PIDSourceType type = PIDSourceType.kRate;
	
	public OneDimensionalMotionProfilerBasic basicProfiler;

	private Elevator() {

		if (EnabledSubsystems.ELEVATOR_ENABLED) {
			elevTalon = CTRECreator.createMasterTalon(RobotMap.ELEVATOR_TALON);
			elevTalonFollow = CTRECreator.createFollowerTalon(RobotMap.ELEVATOR_TALON_FOLLOW, elevTalon.getDeviceID());
	
			elevTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			
			elevTalon.config_kF(VEL_UP_SLOT, 0, DEFAULT_TIMEOUT);
			elevTalon.config_kP(VEL_UP_SLOT, P_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);
			elevTalon.config_kI(VEL_UP_SLOT, I_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);
			elevTalon.config_kD(VEL_UP_SLOT, D_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);
			elevTalon.config_kF(MOTION_MAGIC_UP_SLOT, F_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
			elevTalon.config_kP(MOTION_MAGIC_UP_SLOT, P_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
			elevTalon.config_kI(MOTION_MAGIC_UP_SLOT, I_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
			elevTalon.config_kD(MOTION_MAGIC_UP_SLOT, D_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
			
			elevTalon.config_kF(VEL_DOWN_SLOT, 0, DEFAULT_TIMEOUT);
			elevTalon.config_kP(VEL_DOWN_SLOT, P_VEL_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			elevTalon.config_kI(VEL_DOWN_SLOT, I_VEL_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			elevTalon.config_kD(VEL_DOWN_SLOT, D_VEL_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			elevTalon.config_kF(MOTION_MAGIC_DOWN_SLOT, F_POS_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			elevTalon.config_kP(MOTION_MAGIC_DOWN_SLOT, P_POS_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			elevTalon.config_kI(MOTION_MAGIC_DOWN_SLOT, I_POS_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			elevTalon.config_kD(MOTION_MAGIC_DOWN_SLOT, D_POS_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			
			elevTalon.setNeutralMode(NEUTRAL_MODE_ELEVATOR);
			elevTalon.setInverted(false);
			elevTalon.setSensorPhase(true);
	
			elevTalon.enableVoltageCompensation(true);
			elevTalon.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_ELEVATOR, DEFAULT_TIMEOUT);
	
			elevTalon.enableCurrentLimit(true);
			elevTalon.configPeakCurrentLimit(PEAK_CURRENT_ELEVATOR, DEFAULT_TIMEOUT);
			elevTalon.configPeakCurrentDuration(PEAK_CURRENT_DURATION_ELEVATOR, DEFAULT_TIMEOUT);
			//elevTalon.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_ELEVATOR, DEFAULT_TIMEOUT);
	
			elevTalon.configClosedloopRamp(VOLTAGE_RAMP_RATE_ELEVATOR.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			elevTalon.configOpenloopRamp(VOLTAGE_RAMP_RATE_ELEVATOR.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
	
			elevTalon.configMotionCruiseVelocity((int) MAX_SPEED_ELEVATOR_UP.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND),
							DEFAULT_TIMEOUT);
			elevTalon.configMotionAcceleration((int) MAX_ACCEL_ELEVATOR_UP.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
			
			elevTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, DEFAULT_TIMEOUT);
			elevTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, DEFAULT_TIMEOUT);
	
			elevEncoder = new TalonEncoder(elevTalon, Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV);
	
			elevTalonFollow.setNeutralMode(NEUTRAL_MODE_ELEVATOR);
			
			if (EnabledSubsystems.ELEVATOR_DUMB_ENABLED) {
				elevTalon.set(ControlMode.PercentOutput, 0);
			} else {
				elevTalon.set(ControlMode.Velocity, 0);
			}
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
			return elevEncoder.getVelocity(timePassed);
		return Speed.ZERO;
	}

	/**
	 * @return The current of the elevator talon
	 */
	public double getMasterCurrent() {
		if (elevTalon != null)
			return elevTalon.getOutputCurrent();
		return 0;
	}
	
	public double getFollowerCurrent() {
		if (elevTalonFollow != null) {
			return elevTalonFollow.getOutputCurrent();
		}
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
			
			if (position.sub(getPosition()).greaterThan(Distance.ZERO) || position.sub(getPosition()).equals(Distance.ZERO)) {
				
				elevTalon.selectProfileSlot(MOTION_MAGIC_UP_SLOT, DEFAULT_TIMEOUT);
				
				elevTalon.configMotionCruiseVelocity((int) MAX_SPEED_ELEVATOR_UP.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(
						Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND),
								DEFAULT_TIMEOUT);
				elevTalon.configMotionAcceleration((int) MAX_ACCEL_ELEVATOR_UP.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(
						Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
						DEFAULT_TIMEOUT);
				
			elevTalon.set(ControlMode.MotionMagic, position.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV));
			} else {
				
				elevTalon.selectProfileSlot(MOTION_MAGIC_DOWN_SLOT, DEFAULT_TIMEOUT);
				
				elevTalon.configMotionCruiseVelocity((int) MAX_SPEED_ELEVATOR_DOWN.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(
						Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND),
								DEFAULT_TIMEOUT);
				elevTalon.configMotionAcceleration((int) MAX_ACCEL_ELEVATOR_DOWN.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(
						Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
						DEFAULT_TIMEOUT);
			}
		}
	}
	
	public void setMotorPercentRaw(double percent) {
		if (elevTalon != null) {
			elevTalon.set(ControlMode.PercentOutput, percent);
		}
	}

	/**
	 * @param percent velocity
	 */
	public void setMotorSpeedPercent(double percent) {
		if (elevTalon != null) {
			//elevTalon.set(ControlMode.PercentOutput, percent);
			setMotorSpeed(MAX_SPEED_ELEVATOR_UP.mul(percent));		
		}
	}

	/**
	 * @param speed to set elevator carriage in
	 */
	public void setMotorSpeed(Speed speed) {

		if (elevTalon != null) {
			
			velSetpoint = speed;
			posSetpoint = Distance.ZERO;
			
			if (speed.greaterThan(Speed.ZERO)) {
			
				elevTalon.selectProfileSlot(VEL_UP_SLOT, DEFAULT_TIMEOUT);
				
				elevTalon.config_kF(VEL_UP_SLOT,
						((VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_UP * velSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
								+ MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_UP) * 1023.0)
								/ velSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
										Time.Unit.HUNDRED_MILLISECOND),
						DEFAULT_TIMEOUT);
		
				if (EnabledSubsystems.ELEVATOR_DUMB_ENABLED) {
					elevTalon.set(ControlMode.PercentOutput, velSetpoint.div(MAX_SPEED_ELEVATOR_UP));
				} else {
					elevTalon.set(ControlMode.Velocity,
							velSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND));
				}
				
			} else {
				
				elevTalon.selectProfileSlot(VEL_DOWN_SLOT, DEFAULT_TIMEOUT);
				
				elevTalon.set(ControlMode.PercentOutput, 0);
				
				/*elevTalon.config_kF(VEL_DOWN_SLOT,
						((VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_DOWN * velSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
								+ MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_DOWN) * 1023.0)
								/ velSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
										Time.Unit.HUNDRED_MILLISECOND),
						DEFAULT_TIMEOUT);
		
				if (EnabledSubsystems.ELEVATOR_DUMB_ENABLED) {
					elevTalon.set(ControlMode.PercentOutput, velSetpoint.div(MAX_SPEED_ELEVATOR_DOWN));
				} else {
					elevTalon.set(ControlMode.Velocity,
							velSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV, Time.Unit.HUNDRED_MILLISECOND));
				}*/
				
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
	
	public void enableMotionProfiler(Distance dist, double maxVelPercent, double maxAccelPercent) {
		
		if (dist.greaterThan(Distance.ZERO)) {
			
			basicProfiler = new OneDimensionalMotionProfilerBasic(this, this, kV_UP, kA_UP, kP_UP, kD_UP);
			basicProfiler.setTrajectory(new OneDimensionalTrajectoryRamped(
					dist.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV),
					MAX_SPEED_ELEVATOR_UP.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
							Time.Unit.HUNDRED_MILLISECOND),
					MAX_ACCEL_ELEVATOR_UP.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
							Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND), RAMPED_PROFILE_TIME_MULT_ELEVATOR));
		} else {
			
			basicProfiler = new OneDimensionalMotionProfilerBasic(this, this, kV_DOWN, kA_DOWN, kP_DOWN, kD_DOWN);
			basicProfiler.setTrajectory(new OneDimensionalTrajectoryRamped(
					dist.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV),
					MAX_SPEED_ELEVATOR_DOWN.mul(PROFILE_VEL_PERCENT_ELEVATOR).get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
							Time.Unit.HUNDRED_MILLISECOND),
					MAX_ACCEL_ELEVATOR_DOWN.mul(PROFILE_ACCEL_PERCENT_ELEVATOR).get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
							Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND), RAMPED_PROFILE_TIME_MULT_ELEVATOR));
		}
				
		basicProfiler.enable();
	
	}
	
	public void disableProfiler() {
		basicProfiler.disable();
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		type = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return type;
	}

	@Override
	public double pidGet() {
		if (type == PIDSourceType.kRate) {
			return getInstance().getVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV,
					Time.Unit.HUNDRED_MILLISECOND);
		} else {
			return getInstance().getPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV);
		}
	}

	@Override
	public void pidWrite(double output) {
		setMotorSpeed(MAX_SPEED_ELEVATOR_UP.mul(output));
	}
	
	/**
	 * What is put on SmartDashboard when it's initialized
	 */
	public void smartDashboardInit() {
		if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Elevator Profile Delta Inches: ", 0);
			SmartDashboard.putNumber("Voltage Ramp Rate Elevator Seconds: ",
					VOLTAGE_RAMP_RATE_ELEVATOR.get(Time.Unit.SECOND));
			
			SmartDashboard.putNumber("Elevator kA Up: ", kA_UP);
			SmartDashboard.putNumber("Elevator kP Up: ", kP_UP);
			SmartDashboard.putNumber("Elevator kD Up: ", kD_UP);
			SmartDashboard.putNumber("F Pos Elevator Up: ", F_POS_ELEVATOR_UP);
			SmartDashboard.putNumber("P Pos Elevator Up: ", P_POS_ELEVATOR_UP);
			SmartDashboard.putNumber("I Pos Elevator Up: ", I_POS_ELEVATOR_UP);
			SmartDashboard.putNumber("D Pos Elevator Up: ", D_POS_ELEVATOR_UP);
			SmartDashboard.putNumber("P Vel Elevator Up: ", P_VEL_ELEVATOR_UP);
			SmartDashboard.putNumber("I Vel Elevator Up: ", I_VEL_ELEVATOR_UP);
			SmartDashboard.putNumber("D Vel Elevator Up: ", D_VEL_ELEVATOR_UP);
			
			SmartDashboard.putNumber("Elevator kA Down: ", kA_DOWN);
			SmartDashboard.putNumber("Elevator kP Down: ", kP_DOWN);
			SmartDashboard.putNumber("Elevator kD Down: ", kD_DOWN);
			SmartDashboard.putNumber("F Pos Elevator Down: ", F_POS_ELEVATOR_DOWN);
			SmartDashboard.putNumber("P Pos Elevator Down: ", P_POS_ELEVATOR_DOWN);
			SmartDashboard.putNumber("I Pos Elevator Down: ", I_POS_ELEVATOR_DOWN);
			SmartDashboard.putNumber("D Pos Elevator Down: ", D_POS_ELEVATOR_DOWN);
			SmartDashboard.putNumber("P Vel Elevator Down: ", P_VEL_ELEVATOR_DOWN);
			SmartDashboard.putNumber("I Vel Elevator Down: ", I_VEL_ELEVATOR_DOWN);
			SmartDashboard.putNumber("D Vel Elevator Down: ", D_VEL_ELEVATOR_DOWN);
			
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
			SmartDashboard.putString("Elevator Current: ", getMasterCurrent() + " : " + getFollowerCurrent());
			SmartDashboard.putString("Elevator Velocity vs. Set Velocity: ",
					getVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : "
							+ velSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
			SmartDashboard.putString("Elevator Position vs. Set Position: ",
					getPosition().get(Distance.Unit.INCH) + " : " + posSetpoint.get(Distance.Unit.INCH));
		}
		if (EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
			profilePos = new Distance(SmartDashboard.getNumber("Elevator Profile Delta Inches: ", 0), Distance.Unit.INCH);
			
			F_POS_ELEVATOR_UP = SmartDashboard.getNumber("F Pos Elevator Up: ", F_POS_ELEVATOR_UP);
			elevTalon.config_kF(MOTION_MAGIC_UP_SLOT, F_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
			P_POS_ELEVATOR_UP = SmartDashboard.getNumber("P Pos Elevator Up: ", P_POS_ELEVATOR_UP);
			elevTalon.config_kP(MOTION_MAGIC_UP_SLOT, P_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
			I_POS_ELEVATOR_UP = SmartDashboard.getNumber("I Pos Elevator Up: ", I_POS_ELEVATOR_UP);
			elevTalon.config_kI(MOTION_MAGIC_UP_SLOT, I_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
			D_POS_ELEVATOR_UP = SmartDashboard.getNumber("D Pos Elevator Up: ", D_POS_ELEVATOR_UP);
			elevTalon.config_kD(MOTION_MAGIC_UP_SLOT, D_POS_ELEVATOR_UP, DEFAULT_TIMEOUT);
			P_VEL_ELEVATOR_UP = SmartDashboard.getNumber("P Vel Elevator Up: ", P_VEL_ELEVATOR_UP);
			elevTalon.config_kP(VEL_UP_SLOT, P_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);
			I_VEL_ELEVATOR_UP = SmartDashboard.getNumber("I Vel Elevator Up: ", I_VEL_ELEVATOR_UP);
			elevTalon.config_kI(VEL_UP_SLOT, I_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);
			D_VEL_ELEVATOR_UP = SmartDashboard.getNumber("D Vel Elevator Up: ", D_VEL_ELEVATOR_UP);
			elevTalon.config_kD(VEL_UP_SLOT, D_VEL_ELEVATOR_UP, DEFAULT_TIMEOUT);
			
			F_POS_ELEVATOR_DOWN = SmartDashboard.getNumber("F Pos Elevator Down: ", F_POS_ELEVATOR_DOWN);
			elevTalon.config_kF(MOTION_MAGIC_DOWN_SLOT, F_POS_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			P_POS_ELEVATOR_DOWN = SmartDashboard.getNumber("P Pos Elevator Down: ", P_POS_ELEVATOR_DOWN);
			elevTalon.config_kP(MOTION_MAGIC_DOWN_SLOT, P_POS_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			I_POS_ELEVATOR_DOWN = SmartDashboard.getNumber("I Pos Elevator Down: ", I_POS_ELEVATOR_DOWN);
			elevTalon.config_kI(MOTION_MAGIC_DOWN_SLOT, I_POS_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			D_POS_ELEVATOR_DOWN = SmartDashboard.getNumber("D Pos Elevator Down: ", D_POS_ELEVATOR_DOWN);
			elevTalon.config_kD(MOTION_MAGIC_DOWN_SLOT, D_POS_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			P_VEL_ELEVATOR_DOWN = SmartDashboard.getNumber("P Vel Elevator Down: ", P_VEL_ELEVATOR_DOWN);
			elevTalon.config_kP(VEL_DOWN_SLOT, P_VEL_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			I_VEL_ELEVATOR_DOWN = SmartDashboard.getNumber("I Vel Elevator Down: ", I_VEL_ELEVATOR_DOWN);
			elevTalon.config_kI(VEL_DOWN_SLOT, I_VEL_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			D_VEL_ELEVATOR_DOWN = SmartDashboard.getNumber("D Vel Elevator Down: ", D_VEL_ELEVATOR_DOWN);
			elevTalon.config_kD(VEL_DOWN_SLOT, D_VEL_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			
			PROFILE_VEL_PERCENT_ELEVATOR = SmartDashboard.getNumber("Profile Vel Percent Elevator: ",
					PROFILE_VEL_PERCENT_ELEVATOR);
			PROFILE_ACCEL_PERCENT_ELEVATOR = SmartDashboard.getNumber("Profile Accel Percent Elevator: ",
					PROFILE_ACCEL_PERCENT_ELEVATOR);
			
			kA_UP = SmartDashboard.getNumber("Elevator kA Up: ", kA_UP);
			kP_UP = SmartDashboard.getNumber("Elevator kP Up: ", kP_UP);
			kD_UP = SmartDashboard.getNumber("Elevator kD Up: ", kD_UP);
			
			kA_DOWN = SmartDashboard.getNumber("Elevator kA Down: ", kA_DOWN);
			kP_DOWN = SmartDashboard.getNumber("Elevator kP Down: ", kP_DOWN);
			kD_DOWN = SmartDashboard.getNumber("Elevator kD Down: ", kD_DOWN);
			
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
				new CancelCommand(Elevator.getInstance());
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
