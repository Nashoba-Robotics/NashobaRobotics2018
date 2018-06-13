package edu.nr.robotics.subsystems.elevatorShooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorShooter extends NRSubsystem {

	private static ElevatorShooter singleton;
	
	private TalonSRX elevShooterTalon;
	
	/**
	 * The voltage ramp rate of the elevator shooter. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER = Time.ZERO; // TODO: Test for elevator shooter voltage ramp rate

	/**
	 * The default velocity percents for the elevator shooter
	 */
	public static double VEL_PERCENT_LOW_ELEVATOR_SHOOTER = 0.5;//TODO: Find elevator shooter velocity percents
	public static double VEL_PERCENT_HIGH_ELEVATOR_SHOOTER = 0.9;
	public static double VEL_PERCENT_SCALE_AUTO_ELEVATOR_SHOOTER = 1.0;
	public static double VEL_PERCENT_TRANSFER = 0.5;
	
	/**
	 * Time after elevator shooter motors run to wait before stopping them;
	 */
	public static Time SHOOT_TIME = new Time(0.5, Time.Unit.SECOND); //TODO: Decide on SHOOT_TIME
	
	/**
	 * The current values of the elevator shooter
	 */
	public static final int PEAK_CURRENT_ELEVATOR_SHOOTER = 80;
	public static final int PEAK_CURRENT_DURATION_ELEVATOR_SHOOTER = 250;
	public static final int CONTINUOUS_CURRENT_LIMIT_ELEVATOR_SHOOTER = 40;
	public static final int SENSOR_BROKEN_CURRENT = 10;

	/**
	 * The 100% voltage that is used as a base calculation for all PercentOutputs
	 */
	public static final double VOLTAGE_COMPENSATION_LEVEL_ELEVATOR_SHOOTER = 12;

	/**
	 * The neutral mode of the elevator shooter (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE_ELEVATOR_SHOOTER = NeutralMode.Coast;

	/**
	 * The default timeout of the elevator shooter functions in ms
	 */
	public static final int DEFAULT_TIMEOUT = 0;
	
	public static double shootPercent = 0;
	
	private ElevatorShooter() {
		
		if(EnabledSubsystems.ELEVATOR_SHOOTER_ENABLED) {
		
			elevShooterTalon = CTRECreator.createMasterTalon(RobotMap.ELEVATOR_SHOOTER_TALON);
	
			elevShooterTalon.setNeutralMode(NEUTRAL_MODE_ELEVATOR_SHOOTER);
			elevShooterTalon.setInverted(true);
	
			elevShooterTalon.enableVoltageCompensation(true);
			elevShooterTalon.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_ELEVATOR_SHOOTER, DEFAULT_TIMEOUT);
	
			elevShooterTalon.enableCurrentLimit(true);
			elevShooterTalon.configPeakCurrentLimit(PEAK_CURRENT_ELEVATOR_SHOOTER, DEFAULT_TIMEOUT);
			elevShooterTalon.configPeakCurrentDuration(PEAK_CURRENT_DURATION_ELEVATOR_SHOOTER, DEFAULT_TIMEOUT);
			elevShooterTalon.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_ELEVATOR_SHOOTER, DEFAULT_TIMEOUT);
	
			elevShooterTalon.configClosedloopRamp(VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			elevShooterTalon.configOpenloopRamp(VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
				
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
			elevShooterTalon.set(ControlMode.PercentOutput, percent);	
		}
	}
	
	/**
	 * What is put on SmartDashboard when it's initialized
	 */
	public void smartDashboardInit() {
		if (EnabledSubsystems.ELEVATOR_SHOOTER_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Voltage Ramp Rate Elevator Shooter Seconds: ",
					VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER.get(Time.Unit.SECOND));
			SmartDashboard.putNumber("Elevator Shooter Vel Percent: ", shootPercent);
		}
	}
	
	@Override
	public void smartDashboardInfo() {
		if (EnabledSubsystems.ELEVATOR_SHOOTER_SMARTDASHBOARD_BASIC_ENABLED) {
			SmartDashboard.putNumber("Elevator Shooter Current: ", getCurrent());
		}
		if (EnabledSubsystems.ELEVATOR_SHOOTER_SMARTDASHBOARD_DEBUG_ENABLED) {
			shootPercent = SmartDashboard.getNumber("Elevator Shooter Vel Percent: ", shootPercent);
			VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER = new Time(
					SmartDashboard.getNumber("Voltage Ramp Rate Elevator Shooter Seconds: ",
							VOLTAGE_RAMP_RATE_ELEVATOR_SHOOTER.get(Time.Unit.SECOND)), Time.Unit.SECOND);
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
