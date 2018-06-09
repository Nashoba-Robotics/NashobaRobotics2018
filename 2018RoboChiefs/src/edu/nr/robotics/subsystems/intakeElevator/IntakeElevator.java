package edu.nr.robotics.subsystems.intakeElevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerBasic;
import edu.nr.lib.motionprofiling.OneDimensionalTrajectoryRamped;
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

public class IntakeElevator extends NRSubsystem implements PIDSource, PIDOutput {
	
	private static IntakeElevator singleton;

	private TalonSRX intakeElevTalon;
	
	/**
	 * The encoder ticks per inch moved on the intake elevator
	 */
	public static final double ENC_TICK_PER_INCH_INTAKE_ELEVATOR = 9488.0 / 10.3125;
	
	/**
	 * The max speed of the intake elevator
	 */
	public static final Speed MAX_SPEED_INTAKE_ELEVATOR_UP = new Speed(2.773, Distance.Unit.FOOT, Time.Unit.SECOND);
	public static final Speed MAX_SPEED_INTAKE_ELEVATOR_DOWN = new Speed(4.458, Distance.Unit.FOOT, Time.Unit.SECOND);

	/**
	 * The max acceleration of the intake elevator
	 */
	public static final Acceleration MAX_ACCEL_INTAKE_ELEVATOR_UP = new Acceleration(12, Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND); 
	public static final Acceleration MAX_ACCEL_INTAKE_ELEVATOR_DOWN = new Acceleration(12, Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND);
	
	/**
	 * The acutal real min move voltage of the intake elevator
	 */
	public static final double REAL_MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR_UP = 0.05;
	public static final double REAL_MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR_DOWN = 0.05;
	public static final double INTAKE_FOLD_HOLD_PERCENT = 0.15;
	
	/**
	 * The theoretical minimum voltage needed to move the intake elevator
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR_UP = 0.243; 
	public static final double MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR_DOWN = 0.0;
	
	/**
	 * The slope of voltage over velocity in feet per second
	 */
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_INTAKE_ELEVATOR_UP = 0.273;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_INTAKE_ELEVATOR_DOWN = 0.216;

	/**
	 * The voltage ramp rate of the intake elevator. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_INTAKE_ELEVATOR = new Time(0.05, Time.Unit.SECOND);

	/**
	 * MotionMagic PID values for the intake elevator
	 */
	public static double F_POS_INTAKE_ELEVATOR_UP = 0.077;
	public static double P_POS_INTAKE_ELEVATOR_UP = 0; // TODO: Find intake elevator MagicMotion FPID UP values
	public static double I_POS_INTAKE_ELEVATOR_UP = 0;
	public static double D_POS_INTAKE_ELEVATOR_UP = 0;
	
	public static double F_POS_INTAKE_ELEVATOR_DOWN = 0;
	public static double P_POS_INTAKE_ELEVATOR_DOWN = 0; // TODO: Find intake elevator MagicMotion FPID DOWN values
	public static double I_POS_INTAKE_ELEVATOR_DOWN = 0;
	public static double D_POS_INTAKE_ELEVATOR_DOWN = 0;
	
	/**
	 * Velocity PID values for the intake elevator
	 */
	public static double P_VEL_INTAKE_ELEVATOR_UP = 0.1; 
	public static double I_VEL_INTAKE_ELEVATOR_UP = 0;
	public static double D_VEL_INTAKE_ELEVATOR_UP = 1;
	
	public static double P_VEL_INTAKE_ELEVATOR_DOWN = 0.36; 
	public static double I_VEL_INTAKE_ELEVATOR_DOWN = 0;
	public static double D_VEL_INTAKE_ELEVATOR_DOWN = 3.6;

	/**
	 * The default profiling velocity percent of the intake elevator
	 */
	public static double PROFILE_VEL_PERCENT_INTAKE_ELEVATOR = 0.4;
	public static double DEPLOY_PERCENT_INTAKE_ELEVATOR = 0.3;
	public static double DRIVE_TO_BOTTOM_PERCENT_INTAKE_ELEVATOR = 0.6;
	
	/**
	 * The default profiling acceleration of the intake elevator
	 */
	public static double PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR = 0.9;
	
	/**
	 * The distance from the end of the intake elevator profile at which the stopping
	 * algorithm starts
	 */
	public static final Distance PROFILE_END_POS_THRESHOLD_INTAKE_ELEVATOR = new Distance(1, Distance.Unit.INCH);

	public static final Speed PROFILE_STOP_SPEED_THRESHOLD = new Speed(0.1, Distance.Unit.INCH, Time.Unit.SECOND); //TODO: Decide on this

	public static final Time INTAKE_FLIP_OUT_TIME = new Time(0.5, Time.Unit.SECOND);
	
	/**
	 * Time multiplier for the ramp of the ramped trajectory
	 */
	public static final double RAMPED_PROFILE_TIME_MULT_INTAKE_ELEVATOR = 1000;
	
	/**
	 * The current values of the intake elevator
	 */
	public static final int PEAK_CURRENT_INTAKE_ELEVATOR = 80;
	public static final int PEAK_CURRENT_DURATION_INTAKE_ELEVATOR = 250;
	public static final int CONTINUOUS_CURRENT_LIMIT_INTAKE_ELEVATOR = 40;
	
	/**
	 * The rate of velocity measurements on the intake elevator encoder
	 */
	public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_INTAKE_ELEVATOR = VelocityMeasPeriod.Period_10Ms;

	/**
	 * The number of measurements the intake elevator encoder averages to return a
	 * value
	 */
	public static final double VELOCITY_MEASUREMENT_WINDOW_INTAKE_ELEVATOR = 32;

	/**
	 * The 100% voltage that is used as a base calculation for all
	 * PercentOutputs
	 */
	public static final double VOLTAGE_COMPENSATION_LEVEL_INTAKE_ELEVATOR = 12;

	/**
	 * The neutral mode of the intake elevator (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE_INTAKE_ELEVATOR = NeutralMode.Brake;

	/**
	 * The PID type of the intake elevator. 0 = Primary, 1 = Cascade
	 */
	public static final int PID_TYPE = 0;

	/**
	 * The default timeout of the intake elevator functions in ms
	 */
	public static final int DEFAULT_TIMEOUT = 0;
	
	public static final double kV_UP = 1 / MAX_SPEED_INTAKE_ELEVATOR_UP.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND);
	public static double kA_UP = 0.0004;
	public static double kP_UP = 0;
	public static double kD_UP = 0;
	
	public static final double kV_DOWN = 1 / MAX_SPEED_INTAKE_ELEVATOR_DOWN.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND);
	public static double kA_DOWN = 0.0004;
	public static double kP_DOWN = 0;
	public static double kD_DOWN = 0;
	
	public OneDimensionalMotionProfilerBasic basicProfiler;
	
	public PIDSourceType type = PIDSourceType.kRate;

	/**
	 * The PID slot numbers
	 */
	public static final int VEL_UP_SLOT = 0;
	public static final int MOTION_MAGIC_UP_SLOT = 1;

	public static final int VEL_DOWN_SLOT = 2;
	public static final int MOTION_MAGIC_DOWN_SLOT = 3;
	
	/**
	 * The positions of the intake elevator at each limit switch and at the default
	 * extend height
	 */
	public static final Distance FOLDED_HEIGHT = new Distance(11.9, Distance.Unit.INCH); // TODO: Find FOLDED_HEIGHT
	
	public static final Distance HANDLER_HEIGHT = new Distance(8.5, Distance.Unit.INCH); // TODO: Find HANDLER_HEIGHT
	
	public static final Distance INTAKE_HEIGHT = Distance.ZERO; 
	
	public static final Distance PORTAL_HEIGHT = INTAKE_HEIGHT.add(new Distance(3, Distance.Unit.INCH)); // TODO: Find PORTAL_HEIGHT
	
	public static final Distance BOTTOM_HEIGHT = Distance.ZERO;
	
	public static final double FOLD_CURRENT_SPIKE = 25;
	public static final double HANDLER_CURRENT_SPIKE = 25;
	public static final double HIT_BOTTOM_CURRENT_INTAKE_ELEVATOR = 40;
	
	public Speed velSetpoint = Speed.ZERO;
	public Distance posSetpoint = Distance.ZERO;
	
	public static Distance profileDeltaPos = Distance.ZERO;
	
	public static boolean intakeFolded = false;
	
	private IntakeElevator() {

		if(EnabledSubsystems.INTAKE_ELEVATOR_ENABLED) {
		
			intakeElevTalon = CTRECreator.createMasterTalon(RobotMap.INTAKE_ELEVATOR_TALON);
	
			if (EnabledSubsystems.INTAKE_ELEVATOR_DUMB_ENABLED) {
				intakeElevTalon.set(ControlMode.PercentOutput, 0);
			} else {
				intakeElevTalon.set(ControlMode.Velocity, 0);
			}
	
			intakeElevTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kF(VEL_UP_SLOT, 0, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kP(VEL_UP_SLOT, P_VEL_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kI(VEL_UP_SLOT, I_VEL_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kD(VEL_UP_SLOT, D_VEL_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kF(MOTION_MAGIC_UP_SLOT, F_POS_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kP(MOTION_MAGIC_UP_SLOT, P_POS_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kI(MOTION_MAGIC_UP_SLOT, I_POS_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kD(MOTION_MAGIC_UP_SLOT, D_POS_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			
			intakeElevTalon.config_kF(VEL_DOWN_SLOT, 0, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kP(VEL_DOWN_SLOT, P_VEL_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kI(VEL_DOWN_SLOT, I_VEL_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kD(VEL_DOWN_SLOT, D_VEL_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kF(MOTION_MAGIC_DOWN_SLOT, F_POS_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kP(MOTION_MAGIC_DOWN_SLOT, P_POS_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kI(MOTION_MAGIC_DOWN_SLOT, I_POS_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kD(MOTION_MAGIC_DOWN_SLOT, D_POS_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			
			intakeElevTalon.setNeutralMode(NEUTRAL_MODE_INTAKE_ELEVATOR);
			intakeElevTalon.setInverted(false);
			intakeElevTalon.setSensorPhase(false);
	
			intakeElevTalon.enableVoltageCompensation(true);
			intakeElevTalon.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_INTAKE_ELEVATOR, DEFAULT_TIMEOUT);
	
			intakeElevTalon.enableCurrentLimit(true);
			intakeElevTalon.configPeakCurrentLimit(PEAK_CURRENT_INTAKE_ELEVATOR, DEFAULT_TIMEOUT);
			intakeElevTalon.configPeakCurrentDuration(PEAK_CURRENT_DURATION_INTAKE_ELEVATOR, DEFAULT_TIMEOUT);
			intakeElevTalon.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_INTAKE_ELEVATOR, DEFAULT_TIMEOUT);
	
			intakeElevTalon.configClosedloopRamp(VOLTAGE_RAMP_RATE_INTAKE_ELEVATOR.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			intakeElevTalon.configOpenloopRamp(VOLTAGE_RAMP_RATE_INTAKE_ELEVATOR.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
	
			intakeElevTalon.configMotionCruiseVelocity((int) MAX_SPEED_INTAKE_ELEVATOR_UP.mul(PROFILE_VEL_PERCENT_INTAKE_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND),
							DEFAULT_TIMEOUT);
			intakeElevTalon.configMotionAcceleration((int) MAX_ACCEL_INTAKE_ELEVATOR_UP.mul(PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
			
			intakeElevTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, DEFAULT_TIMEOUT);
		
			smartDashboardInit();
		}
	}
	
	public static IntakeElevator getInstance() {
		if (singleton == null)
			init();
		return singleton;
	}
	
	public synchronized static void init() {
		if (singleton == null) {
			singleton = new IntakeElevator();
			singleton.setJoystickCommand(new IntakeElevatorJoystickCommand());
		}
	}
	
	public Distance getPosition() {
		if (intakeElevTalon != null) {
			return new Distance(intakeElevTalon.getSelectedSensorPosition(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV);
		}
		return Distance.ZERO;
	}
	
	public Speed getVelocity() {
		if (intakeElevTalon != null) {
			return new Speed(intakeElevTalon.getSelectedSensorVelocity(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV,
				Time.Unit.HUNDRED_MILLISECOND);
		}
		return Speed.ZERO;
	}
	
	/**
	 * @return The current of the intake elevator talon
	 */
	public double getCurrent() {
	if (intakeElevTalon != null) {	
			return intakeElevTalon.getOutputCurrent();
		}
	return 0;
	}
	
	/**
	 * @param position
	 *            the absolute position the elevator talon should go to (0+ from
	 *            BOTTOM_HEIGHT up)
	 */
	public void setPosition(Distance position) {
		if (intakeElevTalon != null) {
			posSetpoint = position;
			velSetpoint = Speed.ZERO;
			
			if (position.sub(getPosition()).greaterThan(Distance.ZERO)) {
				intakeElevTalon.selectProfileSlot(MOTION_MAGIC_UP_SLOT, DEFAULT_TIMEOUT);
			
				intakeElevTalon.configMotionCruiseVelocity((int) MAX_SPEED_INTAKE_ELEVATOR_UP.mul(PROFILE_VEL_PERCENT_INTAKE_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND),
							DEFAULT_TIMEOUT);
				intakeElevTalon.configMotionAcceleration((int) MAX_ACCEL_INTAKE_ELEVATOR_UP.mul(PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
			} else {
				
				intakeElevTalon.selectProfileSlot(MOTION_MAGIC_DOWN_SLOT, DEFAULT_TIMEOUT);
				
				intakeElevTalon.configMotionCruiseVelocity((int) MAX_SPEED_INTAKE_ELEVATOR_DOWN.mul(PROFILE_VEL_PERCENT_INTAKE_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND),
							DEFAULT_TIMEOUT);
				intakeElevTalon.configMotionAcceleration((int) MAX_ACCEL_INTAKE_ELEVATOR_DOWN.mul(PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
			}
			intakeElevTalon.set(ControlMode.MotionMagic, position.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV));
		}
	}
	
	public void enableMotionProfiler(Distance dist, double maxVelPercent, double maxAccelPercent) {

		if (dist.greaterThan(Distance.ZERO)) {

			basicProfiler = new OneDimensionalMotionProfilerBasic(this, this, kV_UP, kA_UP, kP_UP, kD_UP);
			basicProfiler.setTrajectory(
					new OneDimensionalTrajectoryRamped(dist.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV),
							MAX_SPEED_INTAKE_ELEVATOR_UP.mul(PROFILE_VEL_PERCENT_INTAKE_ELEVATOR)
									.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND),
							MAX_ACCEL_INTAKE_ELEVATOR_UP.mul(PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR).get(
									Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND,
									Time.Unit.HUNDRED_MILLISECOND),
							RAMPED_PROFILE_TIME_MULT_INTAKE_ELEVATOR));
		} else {

			basicProfiler = new OneDimensionalMotionProfilerBasic(this, this, kV_DOWN, kA_DOWN, kP_DOWN, kD_DOWN);
			basicProfiler.setTrajectory(
					new OneDimensionalTrajectoryRamped(dist.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV),
							MAX_SPEED_INTAKE_ELEVATOR_DOWN.mul(PROFILE_VEL_PERCENT_INTAKE_ELEVATOR)
									.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND),
							MAX_ACCEL_INTAKE_ELEVATOR_DOWN.mul(PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR).get(
									Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND,
									Time.Unit.HUNDRED_MILLISECOND),
							RAMPED_PROFILE_TIME_MULT_INTAKE_ELEVATOR));
		}

		basicProfiler.enable();

	}

	public void disableProfiler() {
		basicProfiler.disable();
	}

	/**
	 * @param percent
	 *            velocity
	 */
	public void setMotorSpeedPercent(double percent) {
		if (intakeElevTalon != null) {
			setMotorSpeed(MAX_SPEED_INTAKE_ELEVATOR_UP.mul(percent));
			//intakeElevTalon.set(ControlMode.PercentOutput, percent);
		}
	}
	
	public void setMotorPercentRaw(double percent) {
		if (intakeElevTalon != null) 
			intakeElevTalon.set(ControlMode.PercentOutput, percent);
	}
	
	/**
	 * @param speed
	 *            to set motor in
	 */
	public void setMotorSpeed(Speed speed) {
		if (intakeElevTalon != null) {
			velSetpoint = speed;
			posSetpoint = Distance.ZERO;
			
			if (speed.greaterThan(Speed.ZERO)) {
				intakeElevTalon.selectProfileSlot(VEL_UP_SLOT, DEFAULT_TIMEOUT);
				intakeElevTalon.config_kF(VEL_UP_SLOT,
					((VOLTAGE_PERCENT_VELOCITY_SLOPE_INTAKE_ELEVATOR_UP * velSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
							+ MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR_UP) * 1023.0)
							/ velSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV,
									Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
	
				if (EnabledSubsystems.INTAKE_ELEVATOR_DUMB_ENABLED) {
					intakeElevTalon.set(ControlMode.PercentOutput, velSetpoint.div(MAX_SPEED_INTAKE_ELEVATOR_UP));
				} else {
					intakeElevTalon.set(ControlMode.Velocity,
						velSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND));
				}
			} else {
				intakeElevTalon.selectProfileSlot(VEL_DOWN_SLOT, DEFAULT_TIMEOUT);
				intakeElevTalon.config_kF(VEL_DOWN_SLOT,
					((VOLTAGE_PERCENT_VELOCITY_SLOPE_INTAKE_ELEVATOR_DOWN * velSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
							+ MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR_DOWN) * 1023.0)
							/ velSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV,
									Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
	
				if (EnabledSubsystems.INTAKE_ELEVATOR_DUMB_ENABLED) {
					intakeElevTalon.set(ControlMode.PercentOutput, velSetpoint.div(MAX_SPEED_INTAKE_ELEVATOR_DOWN));
				} else {
					intakeElevTalon.set(ControlMode.Velocity,
						velSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND));
				}
			}
		}
	}
		
		/**
		 * Sets the time limit going from 0V to 12V in seconds
		 * 
		 * @param time
		 *            going from 0V to 12V
		 */
	public void setVoltageRampRate(Time time) {
		if (intakeElevTalon != null) {
			intakeElevTalon.configOpenloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			intakeElevTalon.configClosedloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
		}
	}
	
	/**
	 * What is put on SmartDashboard when it's initialized
	 */
	public void smartDashboardInit() {
		if (EnabledSubsystems.INTAKE_ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Intake Elevator Profile Delta Inches: ", 0);
			SmartDashboard.putNumber("Voltage Ramp Rate Intake Elevator Seconds: ",
					VOLTAGE_RAMP_RATE_INTAKE_ELEVATOR.get(Time.Unit.SECOND));
			
			SmartDashboard.putNumber("Intake Elevator kA Up: ", kA_UP);
			SmartDashboard.putNumber("Intake Elevator kA Down: ", kA_UP);
			
			SmartDashboard.putNumber("F Pos Intake Elevator Up: ", F_POS_INTAKE_ELEVATOR_UP);
			SmartDashboard.putNumber("P Pos Intake Elevator Up: ", P_POS_INTAKE_ELEVATOR_UP);
			SmartDashboard.putNumber("I Pos Intake Elevator Up: ", I_POS_INTAKE_ELEVATOR_UP);
			SmartDashboard.putNumber("D Pos Intake Elevator Up: ", D_POS_INTAKE_ELEVATOR_UP);
			SmartDashboard.putNumber("P Vel Intake Elevator Up: ", P_VEL_INTAKE_ELEVATOR_UP);
			SmartDashboard.putNumber("I Vel Intake Elevator Up: ", I_VEL_INTAKE_ELEVATOR_UP);
			SmartDashboard.putNumber("D Vel Intake Elevator Up: ", D_VEL_INTAKE_ELEVATOR_UP);
			
			SmartDashboard.putNumber("F Pos Intake Elevator Down: ", F_POS_INTAKE_ELEVATOR_DOWN);
			SmartDashboard.putNumber("P Pos Intake Elevator Down: ", P_POS_INTAKE_ELEVATOR_DOWN);
			SmartDashboard.putNumber("I Pos Intake Elevator Down: ", I_POS_INTAKE_ELEVATOR_DOWN);
			SmartDashboard.putNumber("D Pos Intake Elevator Down: ", D_POS_INTAKE_ELEVATOR_DOWN);
			SmartDashboard.putNumber("P Vel Intake Elevator Down: ", P_VEL_INTAKE_ELEVATOR_DOWN);
			SmartDashboard.putNumber("I Vel Intake Elevator Down: ", I_VEL_INTAKE_ELEVATOR_DOWN);
			SmartDashboard.putNumber("D Vel Intake Elevator Down: ", D_VEL_INTAKE_ELEVATOR_DOWN);
			
			SmartDashboard.putNumber("Profile Vel Percent Intake Elevator: ", PROFILE_VEL_PERCENT_INTAKE_ELEVATOR);
			SmartDashboard.putNumber("Profile Accel Percent Intake Elevator: ", PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR);
		}
	}
	
	/**
	 * What is output or updated on SmartDashboard every time through the loop
	 */
	public void smartDashboardInfo() {
		if (EnabledSubsystems.INTAKE_ELEVATOR_SMARTDASHBOARD_BASIC_ENABLED) {
			SmartDashboard.putNumber("Intake Elevator Current: ", getCurrent());
			SmartDashboard.putString("Intake Elevator Velocity vs. Set Velocity: ",
					getVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : "
							+ velSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
			SmartDashboard.putString("Intake Elevator Position vs. Set Position: ",
					getPosition().get(Distance.Unit.INCH) + " : " + posSetpoint.get(Distance.Unit.INCH));
		}
		if (EnabledSubsystems.INTAKE_ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
			profileDeltaPos = new Distance(SmartDashboard.getNumber("Intake Elevator Profile Delta Inches: ", 0),
					Distance.Unit.INCH);
			
			F_POS_INTAKE_ELEVATOR_UP = SmartDashboard.getNumber("F Pos Intake Elevator Up: ", F_POS_INTAKE_ELEVATOR_UP);
			P_POS_INTAKE_ELEVATOR_UP = SmartDashboard.getNumber("P Pos Intake Elevator Up: ", P_POS_INTAKE_ELEVATOR_UP);
			I_POS_INTAKE_ELEVATOR_UP = SmartDashboard.getNumber("I Pos Intake Elevator Up: ", I_POS_INTAKE_ELEVATOR_UP);
			D_POS_INTAKE_ELEVATOR_UP = SmartDashboard.getNumber("D Pos Intake Elevator Up: ", D_POS_INTAKE_ELEVATOR_UP);
			P_VEL_INTAKE_ELEVATOR_UP = SmartDashboard.getNumber("P Vel Intake Elevator Up: ", P_VEL_INTAKE_ELEVATOR_UP);
			I_VEL_INTAKE_ELEVATOR_UP = SmartDashboard.getNumber("I Vel Intake Elevator Up: ", I_VEL_INTAKE_ELEVATOR_UP);
			D_VEL_INTAKE_ELEVATOR_UP = SmartDashboard.getNumber("D Vel Intake Elevator Up: ", D_VEL_INTAKE_ELEVATOR_UP);
			
			F_POS_INTAKE_ELEVATOR_DOWN = SmartDashboard.getNumber("F Pos Intake Elevator Down: ", F_POS_INTAKE_ELEVATOR_DOWN);
			P_POS_INTAKE_ELEVATOR_DOWN = SmartDashboard.getNumber("P Pos Intake Elevator Down: ", P_POS_INTAKE_ELEVATOR_DOWN);
			I_POS_INTAKE_ELEVATOR_DOWN = SmartDashboard.getNumber("I Pos Intake Elevator Down: ", I_POS_INTAKE_ELEVATOR_DOWN);
			D_POS_INTAKE_ELEVATOR_DOWN = SmartDashboard.getNumber("D Pos Intake Elevator Down: ", D_POS_INTAKE_ELEVATOR_DOWN);
			P_VEL_INTAKE_ELEVATOR_DOWN = SmartDashboard.getNumber("P Vel Intake Elevator Down: ", P_VEL_INTAKE_ELEVATOR_DOWN);
			I_VEL_INTAKE_ELEVATOR_DOWN = SmartDashboard.getNumber("I Vel Intake Elevator Down: ", I_VEL_INTAKE_ELEVATOR_DOWN);
			D_VEL_INTAKE_ELEVATOR_DOWN = SmartDashboard.getNumber("D Vel Intake Elevator Down: ", D_VEL_INTAKE_ELEVATOR_DOWN);
			
			PROFILE_VEL_PERCENT_INTAKE_ELEVATOR = SmartDashboard.getNumber("Profile Vel Percent Intake Elevator: ",
					PROFILE_VEL_PERCENT_INTAKE_ELEVATOR);
			PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR = SmartDashboard.getNumber("Profile Accel Percent Elevator: ",
					PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR);
			
			intakeElevTalon.config_kF(MOTION_MAGIC_UP_SLOT, F_POS_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kP(MOTION_MAGIC_UP_SLOT, P_POS_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kI(MOTION_MAGIC_UP_SLOT, I_POS_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kD(MOTION_MAGIC_UP_SLOT, D_POS_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kP(VEL_UP_SLOT, P_VEL_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kI(VEL_UP_SLOT, I_VEL_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kD(VEL_UP_SLOT, D_VEL_INTAKE_ELEVATOR_UP, DEFAULT_TIMEOUT);
			
			intakeElevTalon.config_kF(MOTION_MAGIC_DOWN_SLOT, F_POS_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kP(MOTION_MAGIC_DOWN_SLOT, P_POS_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kI(MOTION_MAGIC_DOWN_SLOT, I_POS_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kD(MOTION_MAGIC_DOWN_SLOT, D_POS_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kP(VEL_DOWN_SLOT, P_VEL_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kI(VEL_DOWN_SLOT, I_VEL_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kD(VEL_DOWN_SLOT, D_VEL_INTAKE_ELEVATOR_DOWN, DEFAULT_TIMEOUT);
			
			kA_UP = SmartDashboard.getNumber("Intake Elevator kA Up: ", kA_UP);
			kA_DOWN = SmartDashboard.getNumber("Intake Elevator kA Down: ", kA_DOWN);
			
			SmartDashboard.putNumber("Intake Elevator Encoder Ticks: ", intakeElevTalon.getSelectedSensorPosition(PID_TYPE));
		}
	}
	
	public boolean isRevLimitSwitchClosed() {
		return intakeElevTalon.getSensorCollection().isRevLimitSwitchClosed();
	}
	
	@Override
	public void periodic() {
		if (EnabledSubsystems.INTAKE_ELEVATOR_ENABLED) {
			if (intakeElevTalon.getSensorCollection().isRevLimitSwitchClosed()) {
				intakeElevTalon.getSensorCollection().setQuadraturePosition((int) BOTTOM_HEIGHT.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV), DEFAULT_TIMEOUT);
			}	
		}

	}

	public void disable() {
		setMotorSpeedPercent(0);
	}

	@Override
	public void pidWrite(double output) {
		setMotorSpeed(MAX_SPEED_INTAKE_ELEVATOR_UP.mul(output));
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
			return getInstance().getVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV,
					Time.Unit.HUNDRED_MILLISECOND);
		} else {
			return getInstance().getPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV);
		}
	}
	
}
