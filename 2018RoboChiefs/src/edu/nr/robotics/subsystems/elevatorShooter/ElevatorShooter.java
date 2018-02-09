package edu.nr.robotics.subsystems.elevatorShooter;

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

public class ElevatorShooter extends NRSubsystem {

	private static ElevatorShooter singleton;
	
	private TalonSRX elevShooterTalon;
	private TalonEncoder elevShooterEncoder;
	
	/**
	 * The encoder ticks per inch moved on the elevator shooter
	 */
	public static final double ENC_TICK_PER_INCH_ELEVATOR_SHOOTER = 0; //TODO: Find ENC_TICK_PER_INCH_ELEVATOR_SHOOTER
	
	/**
	 * The max speed of the elevator
	 */
	public static final Speed MAX_SPEED_ELEVATOR_SHOOTER = Speed.ZERO; // TODO: Find MAX_SPEED_ELEVATOR_SHOOTER
	
	/**
	 * The minimum voltage needed to move the elevator shooter
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_SHOOTER = 0; // TODO: Find ElevatorShooter voltage velocity curve

	/**
	 * The slope of voltage over velocity in feet per second
	 */
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_SHOOTER = 0;
	
	/**
	 * The voltage ramp rate of the elevator shooter. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER = Time.ZERO; // TODO: Test for elevator shooter voltage ramp rate

	/**
	 * Velocity PID values for the elevator shooter
	 */
	public static double P_VEL_ELEVATOR_SHOOTER = 0; // TODO: Find elevator shooter velocity PID values
	public static double I_VEL_ELEVATOR_SHOOTER = 0;
	public static double D_VEL_ELEVATOR_SHOOTER = 0;

	/**
	 * The default velocity percents for the elevator shooter
	 */
	public static double VEL_PERCENT_LOW_ELEVATOR_SHOOTER = 0;//TODO: Find elevator shooter velocity percents
	public static double VEL_PERCENT_HIGH_ELEVATOR_SHOOTER = 0;
	
	/**
	 * Time after elevator shooter motors run to wait before stopping them;
	 */
	public static Time SHOOT_TIME = Time.ZERO; //TODO: Decide on SHOOT_TIME
	
	/**
	 * The current values of the elevator shooter
	 */
	public static final int PEAK_CURRENT_ELEVATOR_SHOOTER = 80;
	public static final int PEAK_CURRENT_DURATION_ELEVATOR_SHOOTER = 1000;
	public static final int CONTINUOUS_CURRENT_LIMIT_ELEVATOR_SHOOTER = 40;

	/**
	 * The rate of velocity measurements on the elevator shooter encoder
	 */
	public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_ELEVATOR_SHOOTER = VelocityMeasPeriod.Period_10Ms;

	/**
	 * The number of measurements the elevator shooter encoder averages to return a value
	 */
	public static final double VELOCITY_MEASUREMENT_WINDOW_ELEVATOR_SHOOTER = 32;

	/**
	 * The 100% voltage that is used as a base calculation for all PercentOutputs
	 */
	public static final double VOLTAGE_COMPENSATION_LEVEL_ELEVATOR_SHOOTER = 12;

	/**
	 * The neutral mode of the elevator shooter (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE_ELEVATOR_SHOOTER = NeutralMode.Coast;

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
	
	public static double shootPercent = 0;
	
	private ElevatorShooter() {
		
		if(EnabledSubsystems.ELEVATOR_SHOOTER_ENABLED) {
		
			elevShooterTalon = CTRECreator.createMasterTalon(RobotMap.ELEVATOR_SHOOTER_TALON);
	
			if (EnabledSubsystems.ELEVATOR_SHOOTER_DUMB_ENABLED) {
				elevShooterTalon.set(ControlMode.PercentOutput, 0);
			} else {
				elevShooterTalon.set(ControlMode.Velocity, 0);
			}
	
			elevShooterTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			elevShooterTalon.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
			elevShooterTalon.config_kP(VEL_SLOT, P_VEL_ELEVATOR_SHOOTER, DEFAULT_TIMEOUT);
			elevShooterTalon.config_kI(VEL_SLOT, I_VEL_ELEVATOR_SHOOTER, DEFAULT_TIMEOUT);
			elevShooterTalon.config_kD(VEL_SLOT, D_VEL_ELEVATOR_SHOOTER, DEFAULT_TIMEOUT);
			elevShooterTalon.setNeutralMode(NEUTRAL_MODE_ELEVATOR_SHOOTER);
			elevShooterTalon.setInverted(false);
			elevShooterTalon.setSensorPhase(false);
	
			elevShooterTalon.enableVoltageCompensation(true);
			elevShooterTalon.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_ELEVATOR_SHOOTER, DEFAULT_TIMEOUT);
	
			elevShooterTalon.enableCurrentLimit(true);
			elevShooterTalon.configPeakCurrentLimit(PEAK_CURRENT_ELEVATOR_SHOOTER, DEFAULT_TIMEOUT);
			elevShooterTalon.configPeakCurrentDuration(PEAK_CURRENT_DURATION_ELEVATOR_SHOOTER, DEFAULT_TIMEOUT);
			elevShooterTalon.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_ELEVATOR_SHOOTER, DEFAULT_TIMEOUT);
	
			elevShooterTalon.configClosedloopRamp(VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			elevShooterTalon.configOpenloopRamp(VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
	
			elevShooterEncoder = new TalonEncoder(elevShooterTalon, Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV);
		}
		
		smartDashboardInit();
	}
	
	public static ElevatorShooter getInstance() {
		if (singleton == null)
			init();
		return singleton;
	}

	public synchronized static void init() {
		if (singleton == null) {
			singleton = new ElevatorShooter();
		}
	}

	/**
	 * @return The current velocity of the elevator shooter encoders
	 */
	public Speed getVelocity() {
		if (elevShooterTalon != null)
			return new Speed(elevShooterTalon.getSelectedSensorVelocity(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV_SHOOTER,
				Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}

	/**
	 * Gets the historical velocity of the elevator shooter talon
	 * 
	 * @param timePassed
	 *            How long ago to look
	 * @return old velocity of the elevator shooter talon
	 */
	public Speed getHistoricalVelocity(Time timePassed) {
		if (elevShooterTalon != null)
			return new Speed(elevShooterEncoder.getVelocity(timePassed));
		return Speed.ZERO;
	}
	
	/**
	 * @return The current of the elevator shooter talon
	 */
	public double getCurrent() {
		if (elevShooterTalon != null)
			return elevShooterTalon.getOutputCurrent();
		return 0;
	}
	
	/**
	 * @param percent velocity
	 */
	public void setMotorSpeedPercent(double percent) {
		if (elevShooterTalon != null) {
			//setMotorSpeed(MAX_SPEED_ELEVATOR_SHOOTER.mul(percent));
			elevShooterTalon.set(ControlMode.PercentOutput, percent);	
		}
	}

	/**
	 * @param speed to set motor in
	 */
	public void setMotorSpeed(Speed speed) {

		if (elevShooterTalon != null) {

			velSetpoint = speed;
			elevShooterTalon.config_kF(VEL_SLOT,
					((VOLTAGE_PERCENT_VELOCITY_SLOPE_ELEVATOR_SHOOTER * velSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
							+ MIN_MOVE_VOLTAGE_PERCENT_ELEVATOR_SHOOTER) * 1023.0)
							/ velSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV_SHOOTER,
									Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
	
			if (elevShooterTalon.getControlMode() == ControlMode.PercentOutput) {
				elevShooterTalon.set(elevShooterTalon.getControlMode(), velSetpoint.div(MAX_SPEED_ELEVATOR_SHOOTER));
			} else {
				elevShooterTalon.set(elevShooterTalon.getControlMode(),
						velSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_ELEV_SHOOTER, Time.Unit.HUNDRED_MILLISECOND));
			}
		}
	}
	
	/**
	 * What is put on SmartDashboard when it's initialized
	 */
	public void smartDashboardInit() {
		if (EnabledSubsystems.ELEVATOR_SHOOTER_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Voltage Ramp Rate Elevator Shooter Seconds: ",
					VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER.get(Time.Unit.SECOND));
			SmartDashboard.putNumber("P Vel Elevator Shooter: ", P_VEL_ELEVATOR_SHOOTER);
			SmartDashboard.putNumber("I Vel Elevator Shooter: ", I_VEL_ELEVATOR_SHOOTER);
			SmartDashboard.putNumber("D Vel Elevator Shooter: ", D_VEL_ELEVATOR_SHOOTER);
			SmartDashboard.putNumber("Elevator Shooter Vel Percent: ", shootPercent);
		}
	}
	
	@Override
	public void smartDashboardInfo() {
		if (EnabledSubsystems.ELEVATOR_SHOOTER_SMARTDASHBOARD_BASIC_ENABLED) {
			SmartDashboard.putNumber("Elevator Shooter Current: ", getCurrent());
			SmartDashboard.putNumber("Elevator Shooter Velocity vs Set Velocity: ", velSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		}
		if (EnabledSubsystems.ELEVATOR_SHOOTER_SMARTDASHBOARD_DEBUG_ENABLED) {
			shootPercent = SmartDashboard.getNumber("Elevator Shooter Vel Percent: ", shootPercent);
			VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER = new Time(
					SmartDashboard.getNumber("Voltage Ramp Rate Elevator Shooter Seconds: ",
							VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER.get(Time.Unit.SECOND)), Time.Unit.SECOND);
			P_VEL_ELEVATOR_SHOOTER = SmartDashboard.getNumber("P Vel Elevator Shooter: ", P_VEL_ELEVATOR_SHOOTER);
			I_VEL_ELEVATOR_SHOOTER = SmartDashboard.getNumber("I Vel Elevator Shooter: ", I_VEL_ELEVATOR_SHOOTER);
			D_VEL_ELEVATOR_SHOOTER = SmartDashboard.getNumber("D Vel Elevator Shooter: ", D_VEL_ELEVATOR_SHOOTER);
			
			SmartDashboard.putNumber("Elevator Shooter Encoder Ticks: ", elevShooterTalon.getSelectedSensorPosition(PID_TYPE));
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
