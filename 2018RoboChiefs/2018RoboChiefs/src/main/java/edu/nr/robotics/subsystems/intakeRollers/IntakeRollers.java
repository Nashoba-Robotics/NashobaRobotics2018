package edu.nr.robotics.subsystems.intakeRollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeRollers extends NRSubsystem {
	
	private static IntakeRollers singleton;
	
	private TalonSRX intakeRollersLeft, intakeRollersRight;
	
	/**
	 * The voltage ramp rate of the elevator shooter. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_INTAKE_ROLLERS = new Time(0.05, Time.Unit.SECOND); //ramp rate in seconds

	/**
	 * The optimal velocity percent for the intake rollers
	 */
	public static double VEL_PERCENT_HIGH_INTAKE_ROLLERS = 0.7;
	public static double VEL_PERCENT_LOW_INTAKE_ROLLERS = 0.5;
	public static double VEL_PERCENT_TRANSFER_INTAKE_ROLLERS = 0.6;
	public static double VEL_PERCENT_PORTAL_INTAKE_ROLLERS = 0.60;//0.90; //TODO: Find VEL_PERCENT_PORTAL_INTAKE_ROLLERS
	public static double VEL_PERCENT_PUKE_INTAKE_ROLLERS = 0.20; //TODO: Find VEL_PERCENT_PUKE_INTAKE_ROLLERS
	
	public static Time SCORE_TIME = new Time(0.5, Time.Unit.SECOND);
	
	/**
	 * The current values of the elevator shooter
	 */
	public static final int PEAK_CURRENT_INTAKE_ROLLERS = 80;
	public static final int PEAK_CURRENT_DURATION_INTAKE_ROLLERS = 250;	
	public static final int CONTINUOUS_CURRENT_LIMIT_INTAKE_ROLLERS = 40;
	
	public static final int CURRENT_PEAK_INTAKE_ROLLERS = 40;
	
	/**
	 * The 100% voltage that is used as a base calculation for all PercentOutputs
	 */
	public static final double VOLTAGE_COMPENSATION_LEVEL_INTAKE_ROLLERS = 12;
	
	/**
	 * The neutral mode of the elevator shooter (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE_INTAKE_ROLLERS = NeutralMode.Brake;
	
	/**
	 * The default timeout of the elevator shooter functions in ms
	 */
	public static final int DEFAULT_TIMEOUT = 0;
	
	public double percentHighSetpoint = 0;
	public double percentLowSetpoint = 0;
	
	private IntakeRollers() {

		if(EnabledSubsystems.INTAKE_ROLLERS_ENABLED) {
		
			intakeRollersLeft = CTRECreator.createMasterTalon(RobotMap.INTAKE_ROLLERS_LEFT);
			intakeRollersRight = CTRECreator.createMasterTalon(RobotMap.INTAKE_ROLLERS_RIGHT);
					
			intakeRollersLeft.setNeutralMode(NEUTRAL_MODE_INTAKE_ROLLERS);
			intakeRollersLeft.setInverted(true);
	
			intakeRollersLeft.enableVoltageCompensation(true);
			intakeRollersLeft.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
	
			intakeRollersLeft.enableCurrentLimit(true);
			intakeRollersLeft.configPeakCurrentLimit(PEAK_CURRENT_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
			intakeRollersLeft.configPeakCurrentDuration(PEAK_CURRENT_DURATION_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
			intakeRollersLeft.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
	
			intakeRollersLeft.configClosedloopRamp(VOLTAGE_RAMP_RATE_INTAKE_ROLLERS.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			intakeRollersLeft.configOpenloopRamp(VOLTAGE_RAMP_RATE_INTAKE_ROLLERS.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			
			intakeRollersRight.setNeutralMode(NEUTRAL_MODE_INTAKE_ROLLERS);
			intakeRollersRight.setInverted(false);
	
			intakeRollersRight.enableVoltageCompensation(true);
			intakeRollersRight.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
	
			intakeRollersRight.enableCurrentLimit(true);
			intakeRollersRight.configPeakCurrentLimit(PEAK_CURRENT_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
			intakeRollersRight.configPeakCurrentDuration(PEAK_CURRENT_DURATION_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
			intakeRollersRight.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_INTAKE_ROLLERS, DEFAULT_TIMEOUT);
	
			intakeRollersRight.configClosedloopRamp(VOLTAGE_RAMP_RATE_INTAKE_ROLLERS.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			intakeRollersRight.configOpenloopRamp(VOLTAGE_RAMP_RATE_INTAKE_ROLLERS.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);

			
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
	 * @return The current of the intake rollers left
	 */
	public double getCurrentLeft() {
		if (intakeRollersLeft != null)
			return intakeRollersLeft.getOutputCurrent();
		return 0;
	}
	
	/**
	 * @return The current of the intake rollers right
	 */
	public double getCurrentRight() {
		if (intakeRollersRight != null) {
			return intakeRollersRight.getOutputCurrent();
		}
		return 0;
	}
	
	/**
	 * @param percent velocity
	 */
	public void setMotorSpeedPercent(double percentHigh, double percentLow) {
		if (intakeRollersLeft != null && intakeRollersRight != null) {
			
			percentHighSetpoint = percentHigh;
			percentLowSetpoint = percentLow;
			
			intakeRollersLeft.set(ControlMode.PercentOutput, percentHigh);
			intakeRollersRight.set(ControlMode.PercentOutput, percentLow);
		}
	}
	
	/**
	 * What is put on SmartDashboard when it's initialized
	 */
	public void smartDashboardInit() {
		if (EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Intake Rollers Vel Percent High: ", VEL_PERCENT_HIGH_INTAKE_ROLLERS);
			SmartDashboard.putNumber("Intake Rollers Vel Percent Low: ", VEL_PERCENT_LOW_INTAKE_ROLLERS);
		}
	}
	
	@Override
	public void smartDashboardInfo() {
		if (EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_BASIC_ENABLED) {
			SmartDashboard.putNumberArray("Intake Rollers Current", new double[] {getCurrentLeft(), getCurrentRight()});
			//SmartDashboard.putNumber("Intake Rollers Current Left: ", getCurrentLeft());
			//SmartDashboard.putNumber("Intake Rollers Current Right: ", getCurrentRight());
		}
		if (EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_DEBUG_ENABLED) {
			VEL_PERCENT_HIGH_INTAKE_ROLLERS = SmartDashboard.getNumber("Intake Rollers Vel Percent High: ", VEL_PERCENT_HIGH_INTAKE_ROLLERS);
			VEL_PERCENT_LOW_INTAKE_ROLLERS = SmartDashboard.getNumber("Intake Rollers Vel Percent Low: ", VEL_PERCENT_LOW_INTAKE_ROLLERS);
		}
	}
	
	@Override
	public void periodic() {
		
	}

	@Override
	public void disable() {
		setMotorSpeedPercent(0, 0);		
	}

}
