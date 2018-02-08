package edu.nr.robotics.subsystems.intakeRollers;

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

public class IntakeRollers extends NRSubsystem {
	
	private static IntakeRollers singleton;
	
	private TalonSRX intakeRollersMaster, intakeRollersFollower;
	private TalonEncoder intakeRollersEncoder;
	
	//TODO: determine ALL for real
	
	/**
	 * The encoder ticks per inch moved on the intake rollers
	 */
	public static final double ENC_TICK_PER_INCH_INTAKE_ROLLERS = 0;
	
	/**
	 * The max speed of the intake rollers
	 */
	public static final Speed INTAKE_ROLLERS_MAX_SPEED = Speed.ZERO;
	
	/**
	 * The minimum voltage needed to move the elevator shooter
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ROLLERS = 0;
	
	/**
	 * The slope of voltage over velocity in feet per second
	 */
	public static final double VOLTAGE_VELOCITY_SLOPE_INTAKE_ROLLERS = 0;
	
	/**
	 * The voltage ramp rate of the elevator shooter. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_INTAKE_ROLLERS = Time.ZERO; //ramp rate in seconds
	
	/**
	 * Velocity PID values for the elevator shooter
	 */
	public static double P_VEL_INTAKE_ROLLERS = 0;
	public static double I_VEL_INTAKE_ROLLERS = 0;
	public static double D_VEL_INTAKE_ROLLERS = 0;
	
	/**
	 * The optimal velocity percent for the intake rollers
	 */
	public static double VEL_PERCENT_INTAKE_ROLLERS = 0;
	
	/**
	 * The current values of the elevator shooter
	 */
	public static final int PEAK_CURRENT_INTAKE_ROLLERS = 80;
	public static final int PEAK_CURRENT_DURATION_INTAKE_ROLLERS = 1000;	
	public static final int CONTINUOUS_CURRENT_LIMIT_INTAKE_ROLLERS = 40;
	
	/**
	 * The rate of velocity measurements on the elevator shooter encoder
	 */
	public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_INTAKE_ROLLERS = VelocityMeasPeriod.Period_10Ms;	
	
	/**
	 * The number of measurements the elevator shooter encoder averages to return a value
	 */
	public static final double VELOCITY_MEASUREMENT_WINDOW_INTAKE_ROLLERS = 32;	
	
	/**
	 * The 100% voltage that is used as a base calculation for all PercentOutputs
	 */
	public static final double VOLTAGE_COMPENSATION_LEVEL_INTAKE_ROLLERS = 12;
	
	/**
	 * The neutral mode of the elevator shooter (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE_INTAKE_ROLLERS = NeutralMode.Brake;
	
	/**
	 * The PID type of the elevator shooter. 0 = Primary, 1 = Cascade
	 */
	public static final int PID_TYPE = 0;
	
	/**
	 * The default timeout of the elevator shooter functions in ms
	 */
	public static final int DEFAULT_TIMEOUT = 0;
	
	/**
	 * The PID slot numbers
	 */
	public static final int VEL_SLOT = 0;

	private Speed velSetpoint = Speed.ZERO;
	
	private IntakeRollers() {

		if(EnabledSubsystems.INTAKE_ROLLERS_ENABLED) {
		
			intakeRollersMaster = CTRECreator.createMasterTalon(RobotMap.INTAKE_ROLLERS_MASTER);
			intakeRollersFollower = CTRECreator.createFollowerTalon(RobotMap.INTAKE_ROLLERS_FOLLOW, RobotMap.INTAKE_ROLLERS_MASTER);
	
			if (EnabledSubsystems.INTAKE_ROLLERS_DUMB_ENABLED) {
				intakeRollersMaster.set(ControlMode.PercentOutput, 0);
			} else {
				intakeRollersMaster.set(ControlMode.Velocity, 0);
			}
	
			intakeRollersMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			intakeRollersMaster.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
			intakeRollersMaster.config_kP(VEL_SLOT, P_VEL_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
			intakeRollersMaster.config_kI(VEL_SLOT, I_VEL_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
			intakeRollersMaster.config_kD(VEL_SLOT, D_VEL_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
			intakeRollersMaster.setNeutralMode(NEUTRAL_MODE_INTAKE_ROLLERS);
			intakeRollersMaster.setInverted(false);
			intakeRollersMaster.setSensorPhase(false);
	
			intakeRollersMaster.enableVoltageCompensation(true);
			intakeRollersMaster.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
	
			intakeRollersMaster.enableCurrentLimit(true);
			intakeRollersMaster.configPeakCurrentLimit(PEAK_CURRENT_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
			intakeRollersMaster.configPeakCurrentDuration(PEAK_CURRENT_DURATION_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
			intakeRollersMaster.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
	
			intakeRollersMaster.configClosedloopRamp(VOLTAGE_RAMP_RATE_INTAKE_ROLLERS.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			intakeRollersMaster.configOpenloopRamp(VOLTAGE_RAMP_RATE_INTAKE_ROLLERS.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
	
			intakeRollersFollower.setNeutralMode(NEUTRAL_MODE_INTAKE_ROLLERS);
			
			intakeRollersEncoder = new TalonEncoder(intakeRollersMaster, Distance.Unit.MAGNETIC_ENCODER_TICK_INTAKE_ROLLERS);
		}
		
		smartDashboardInit();
	}
	
	public static IntakeRollers getInstance() {
		if (singleton == null)
			init();
		return singleton;
	}
	
	public synchronized static void init() {
		if (singleton == null) {
			singleton = new IntakeRollers();
		}
	}
	
	/**
	 * @return The current velocity of the intake rollers encoder
	 */
	public Speed getVelocity() {
		if (intakeRollersMaster != null)
			return new Speed(intakeRollersMaster.getSelectedSensorVelocity(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV_SHOOTER,
				Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	/**
	 * Gets the historical velocity of the intake rollers master
	 * 
	 * @param timePassed
	 *            How long ago to look
	 * @return old velocity of the intake rollers master
	 */
	public Speed getHistoricalVelocity(Time timePassed) {
		if (intakeRollersMaster != null)
			return new Speed(intakeRollersEncoder.getVelocity(timePassed));
		return Speed.ZERO;
	}
	
	/**
	 * @return The current of the intake rollers master
	 */
	public double getCurrent() {
		if (intakeRollersMaster != null)
			return intakeRollersMaster.getOutputCurrent();
		return 0;
	}
	
	/**
	 * @param percent velocity
	 */
	public void setMotorSpeedPercent(double percent) {
		setMotorSpeed(INTAKE_ROLLERS_MAX_SPEED.mul(percent));
	}
	
	/**
	 * @param speed to set motor in
	 */
	public void setMotorSpeed(Speed speed) {

		if (intakeRollersMaster != null) {

			velSetpoint = speed;
			intakeRollersMaster.config_kF(VEL_SLOT,
					((VOLTAGE_VELOCITY_SLOPE_INTAKE_ROLLERS * velSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
							+ MIN_MOVE_VOLTAGE_PERCENT_INTAKE_ROLLERS) * 1023.0)
							/ velSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV_SHOOTER,
									Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
	
			if (intakeRollersMaster.getControlMode() == ControlMode.PercentOutput) {
				intakeRollersMaster.set(intakeRollersMaster.getControlMode(), velSetpoint.div(INTAKE_ROLLERS_MAX_SPEED));
			} else {
				intakeRollersMaster.set(intakeRollersMaster.getControlMode(),
						velSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV_SHOOTER, Time.Unit.HUNDRED_MILLISECOND));
			}
		}
	}
	
	/**
	 * What is put on SmartDashboard when it's initialized
	 */
	public void smartDashboardInit() {
		if (EnabledSubsystems.ELEVATOR_SHOOTER_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Voltage Ramp Rate Intake Rollers Seconds: ",
					VOLTAGE_RAMP_RATE_INTAKE_ROLLERS.get(Time.Unit.SECOND));
			SmartDashboard.putNumber("P Vel Intake Rollers: ", P_VEL_INTAKE_ROLLERS);
			SmartDashboard.putNumber("I Vel Intake Rollers: ", I_VEL_INTAKE_ROLLERS);
			SmartDashboard.putNumber("D Vel Intake Rollers: ", D_VEL_INTAKE_ROLLERS);
			SmartDashboard.putNumber("Intake Rollers Vel Percent: ", VEL_PERCENT_INTAKE_ROLLERS);
		}
	}
	
	@Override
	public void smartDashboardInfo() {
		if (EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_BASIC_ENABLED) {
			SmartDashboard.putNumber("Intake Rollers Current: ", getCurrent());
			SmartDashboard.putNumber("Intake Rollers Velocity vs Set Velocity: ", velSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		}
		if (EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_DEBUG_ENABLED) {
			VEL_PERCENT_INTAKE_ROLLERS = SmartDashboard.getNumber("Intake Rollers Vel Percent: ", VEL_PERCENT_INTAKE_ROLLERS);
			VOLTAGE_RAMP_RATE_INTAKE_ROLLERS = new Time(
					SmartDashboard.getNumber("Voltage Ramp Rate Intake Rollers Seconds: ",
							VOLTAGE_RAMP_RATE_INTAKE_ROLLERS.get(Time.Unit.SECOND)), Time.Unit.SECOND);
			P_VEL_INTAKE_ROLLERS = SmartDashboard.getNumber("P Vel Intake Rollers: ", P_VEL_INTAKE_ROLLERS);
			I_VEL_INTAKE_ROLLERS = SmartDashboard.getNumber("I Vel Intake Rollers: ", I_VEL_INTAKE_ROLLERS);
			D_VEL_INTAKE_ROLLERS = SmartDashboard.getNumber("D Vel Intake Rollers: ", D_VEL_INTAKE_ROLLERS);
			
			SmartDashboard.putNumber("Intake Rollers Encoder Ticks: ", intakeRollersMaster.getSelectedSensorPosition(PID_TYPE));
		}
	}
	
	@Override
	public void periodic() {
		
	}

	@Override
	public void disable() {
		setMotorSpeedPercent(0);		
	}

}
