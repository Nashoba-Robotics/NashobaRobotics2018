package edu.nr.robotics.subsystems.intakeElevator;

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

public class IntakeElevator extends NRSubsystem {
	private static IntakeElevator singleton;

	private TalonSRX intakeElevTalon;
	private TalonEncoder intakeElevEncoder;

	/**
	 * The encoder ticks per inch moved on the intake elevator
	 */
	public static final double ENC_TICK_PER_INCH_INTAKE_ELEVATOR = 0; // TODO: Find ENC_TICK_PER_INCH_INTAKE_ELEVATOR

	/**
	 * The max speed of the intake elevator
	 */
	public static final Speed MAX_SPEED_INTAKE_ELEVATOR = Speed.ZERO; // TODO: Find MAX_SPEED_INTAKE_ELEVATOR

	/**
	 * The max acceleration of the intake elevator
	 */
	public static final Acceleration MAX_ACCEL_INTAKE_ELEVATOR = Acceleration.ZERO; // TODO: Find MAX_ACCEL_INTAKE_ELEVATOR

	/**
	 * The minimum voltage needed to move the intake elevator
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR = 0; // TODO: Find Elevator voltage velocity curve

	/**
	 * The slope of voltage over velocity in feet per second
	 */
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_INTAKE_ELEVATOR = 0;

	/**
	 * The voltage ramp rate of the intake elevator. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_INTAKE_ELEVATOR = Time.ZERO; // TODO: Test for intake elevator voltage ramp rate

	/**
	 * MotionMagic PID values for the intake elevator
	 */
	public static double F_POS_INTAKE_ELEVATOR = 1.00 * 1023 / MAX_SPEED_INTAKE_ELEVATOR.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND);
	public static double P_POS_INTAKE_ELEVATOR = 0; // TODO: Find intake elevator MagicMotion PID values
	public static double I_POS_INTAKE_ELEVATOR = 0;
	public static double D_POS_INTAKE_ELEVATOR = 0;

	/**
	 * Velocity PID values for the intake elevator
	 */
	public static double P_VEL_INTAKE_ELEVATOR = 0; // TODO: Find intake elevator velocity PID values
	public static double I_VEL_INTAKE_ELEVATOR = 0;
	public static double D_VEL_INTAKE_ELEVATOR = 0;

	/**
	 * The default profiling velocity percent of the intake elevator
	 */
	public static double PROFILE_VEL_PERCENT_INTAKE_ELEVATOR = 0; // TODO: Decide on PROFILE_VEL_PERCENT_INTAKE_ELEVATOR

	/**
	 * The default profiling acceleration of the intake elevator
	 */
	public static double PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR = 0; // TODO: Decide on PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR

	/**
	 * The distance from the end of the intake elevator profile at which the stopping
	 * algorithm starts
	 */
	public static final Distance PROFILE_END_POS_THRESHOLD_INTAKE_ELEVATOR = Distance.ZERO; // TODO: Decide on PROFILE_END_POS_THRESHOLD_INTAKE_ELEVATOR

	/**
	 * The change in position intake elevator within for
	 * PROFILE_TIME_POS_THRESHOLD_INTAKE_ELEVATOR before stopping profile
	 */
	public static final Distance PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR = Distance.ZERO; // TODO: Decide on PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR
	public static final Time PROFILE_DELTA_TIME_THRESHOLD_INTAKE_ELEVATOR = new Time(10, Time.Unit.MILLISECOND);

	/**
	 * The current values of the intake elevator
	 */
	public static final int PEAK_CURRENT_INTAKE_ELEVATOR = 80;
	public static final int PEAK_CURRENT_DURATION_INTAKE_ELEVATOR = 1000;
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

	/**
	 * The PID slot numbers
	 */
	public static final int VEL_SLOT = 0;
	public static final int MOTION_MAGIC_SLOT = 1;

	/**
	 * The positions of the intake elevator at each limit switch and at the default
	 * extend height
	 */
	public static final Distance FOLDED_HEIGHT = Distance.ZERO; // TODO: Find FOLDED_HEIGHT
	
	public static final Distance HANDLER_HEIGHT = Distance.ZERO; // TODO: Find HANDLER_HEIGHT
	
	public static final Distance INTAKE_HEIGHT = Distance.ZERO; // TODO: Find INTAKE_HEIGHT
	
	public static final Distance BOTTOM_HEIGHT = Distance.ZERO;

	
	public Speed velSetpoint = Speed.ZERO;
	public Distance posSetpoint = Distance.ZERO;
	
	public static Distance profileDeltaPos = Distance.ZERO;
	
	
	private IntakeElevator() {

		if(EnabledSubsystems.INTAKE_ELEVATOR_ENABLED) {
		
			intakeElevTalon = CTRECreator.createMasterTalon(RobotMap.INTAKE_ELEVATOR_TALON);
	
			if (EnabledSubsystems.INTAKE_ELEVATOR_DUMB_ENABLED) {
				intakeElevTalon.set(ControlMode.PercentOutput, 0);
			} else {
				intakeElevTalon.set(ControlMode.Velocity, 0);
			}
	
			intakeElevTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kP(VEL_SLOT, P_VEL_INTAKE_ELEVATOR, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kI(VEL_SLOT, I_VEL_INTAKE_ELEVATOR, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kD(VEL_SLOT, D_VEL_INTAKE_ELEVATOR, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kF(MOTION_MAGIC_SLOT, F_POS_INTAKE_ELEVATOR, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kP(MOTION_MAGIC_SLOT, P_POS_INTAKE_ELEVATOR, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kI(MOTION_MAGIC_SLOT, I_POS_INTAKE_ELEVATOR, DEFAULT_TIMEOUT);
			intakeElevTalon.config_kD(MOTION_MAGIC_SLOT, D_POS_INTAKE_ELEVATOR, DEFAULT_TIMEOUT);
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
	
			intakeElevTalon.configMotionCruiseVelocity((int) MAX_SPEED_INTAKE_ELEVATOR.mul(PROFILE_VEL_PERCENT_INTAKE_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND),
							DEFAULT_TIMEOUT);
			intakeElevTalon.configMotionAcceleration((int) MAX_ACCEL_INTAKE_ELEVATOR.mul(PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
			
			intakeElevTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, DEFAULT_TIMEOUT);
			intakeElevTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, DEFAULT_TIMEOUT);
	
			intakeElevEncoder = new TalonEncoder(intakeElevTalon, Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV);
	
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
	
	public Distance getHistoricalPosition(Time timePassed) {
		if (intakeElevTalon != null) {
			return intakeElevEncoder.getPosition(timePassed);
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
	
	public Speed getHistoricalVelocity(Time timePassed) {
		if (intakeElevTalon != null) {
			return new Speed(intakeElevEncoder.getVelocity(timePassed));
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
			intakeElevTalon.configMotionCruiseVelocity((int) MAX_SPEED_INTAKE_ELEVATOR.mul(PROFILE_VEL_PERCENT_INTAKE_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND),
							DEFAULT_TIMEOUT);
			intakeElevTalon.configMotionAcceleration((int) MAX_ACCEL_INTAKE_ELEVATOR.mul(PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR).get(
					Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
			intakeElevTalon.set(ControlMode.MotionMagic, position.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV));
		}
	}
	
	/**
	 * @param percent
	 *            velocity
	 */
	public void setMotorSpeedPercent(double percent) {
		if (intakeElevTalon != null) {
			setMotorSpeed(MAX_SPEED_INTAKE_ELEVATOR.mul(percent));
		}
	}
	
	/**
	 * @param speed
	 *            to set motor in
	 */
	public void setMotorSpeed(Speed speed) {
		if (intakeElevTalon != null) {
			velSetpoint = speed;
			posSetpoint = Distance.ZERO;
			intakeElevTalon.config_kF(VEL_SLOT,
				((VOLTAGE_PERCENT_VELOCITY_SLOPE_INTAKE_ELEVATOR * velSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
						+ MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ELEVATOR) * 1023.0)
						/ velSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV,
								Time.Unit.HUNDRED_MILLISECOND),
				DEFAULT_TIMEOUT);

			if (intakeElevTalon.getControlMode() == ControlMode.PercentOutput) {
				intakeElevTalon.set(intakeElevTalon.getControlMode(), velSetpoint.div(MAX_SPEED_INTAKE_ELEVATOR));
			} else {
				intakeElevTalon.set(intakeElevTalon.getControlMode(),
					velSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ELEV, Time.Unit.HUNDRED_MILLISECOND));
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
			SmartDashboard.putNumber("P Pos Intake Elevator: ", P_POS_INTAKE_ELEVATOR);
			SmartDashboard.putNumber("I Pos Intake Elevator: ", I_POS_INTAKE_ELEVATOR);
			SmartDashboard.putNumber("D Pos Intake Elevator: ", D_POS_INTAKE_ELEVATOR);
			SmartDashboard.putNumber("P Vel Intake Elevator: ", P_VEL_INTAKE_ELEVATOR);
			SmartDashboard.putNumber("I Vel Intake Elevator: ", I_VEL_INTAKE_ELEVATOR);
			SmartDashboard.putNumber("D Vel Intake Elevator: ", D_VEL_INTAKE_ELEVATOR);
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
			P_POS_INTAKE_ELEVATOR = SmartDashboard.getNumber("P Pos Intake Elevator: ", P_POS_INTAKE_ELEVATOR);
			I_POS_INTAKE_ELEVATOR = SmartDashboard.getNumber("I Pos Intake Elevator: ", I_POS_INTAKE_ELEVATOR);
			D_POS_INTAKE_ELEVATOR = SmartDashboard.getNumber("D Pos Intake Elevator: ", D_POS_INTAKE_ELEVATOR);
			P_VEL_INTAKE_ELEVATOR = SmartDashboard.getNumber("P Vel Intake Elevator: ", P_VEL_INTAKE_ELEVATOR);
			I_VEL_INTAKE_ELEVATOR = SmartDashboard.getNumber("I Vel Intake Elevator: ", I_VEL_INTAKE_ELEVATOR);
			D_VEL_INTAKE_ELEVATOR = SmartDashboard.getNumber("D Vel Intake Elevator: ", D_VEL_INTAKE_ELEVATOR);
			PROFILE_VEL_PERCENT_INTAKE_ELEVATOR = SmartDashboard.getNumber("Profile Vel Percent Intake Elevator: ",
					PROFILE_VEL_PERCENT_INTAKE_ELEVATOR);
			PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR = SmartDashboard.getNumber("Profile Accel Percent Elevator: ",
					PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR);
			
			SmartDashboard.putNumber("Intake Elevator Encoder Ticks: ", intakeElevTalon.getSelectedSensorPosition(PID_TYPE));
		}
	}
	
	public boolean isFwdLimitSwitchClosed() {
		return intakeElevTalon.getSensorCollection().isFwdLimitSwitchClosed();
	}
	
	public boolean isRevLimitSwitchClosed() {
		return intakeElevTalon.getSensorCollection().isRevLimitSwitchClosed();
	}
	
	@Override
	public void periodic() {
		if (EnabledSubsystems.INTAKE_ELEVATOR_ENABLED) {
			if(intakeElevTalon.getSensorCollection().isFwdLimitSwitchClosed()) {
				intakeElevTalon.getSensorCollection().setQuadraturePosition((int) FOLDED_HEIGHT.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV), DEFAULT_TIMEOUT);
			}
			if (intakeElevTalon.getSensorCollection().isRevLimitSwitchClosed()) {
				intakeElevTalon.getSensorCollection().setQuadraturePosition((int) BOTTOM_HEIGHT.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV), DEFAULT_TIMEOUT);
			}	
		}

	}

	public void disable() {
		setMotorSpeedPercent(0);
	}
}
