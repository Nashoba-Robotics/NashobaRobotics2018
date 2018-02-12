package edu.nr.robotics.subsystems.cubeHandler;

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

public class CubeHandler extends NRSubsystem {
	
private static CubeHandler singleton;
	
	private TalonSRX cubeHandlerTalon;
		
	/**
	 * The voltage ramp rate of the cube handler. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_CUBE_HANDLER = Time.ZERO; // TODO: Test for cube handler voltage ramp rate

	/**
	 * The default velocity percent for the cube handler
	 */
	public static double VEL_PERCENT_CUBE_HANDLER = 0;//TODO: Find cube handler velocity percent
	
	/**
	 * The time to run the cube handlers so the cube exits the robot
	 */
	public static Time SCORE_TIME = Time.ZERO; //TODO: Find cube handler SCORE_TIME
	
	/**
	 * The current values of the cube handler
	 */
	public static final int PEAK_CURRENT_CUBE_HANDLER = 80;
	public static final int PEAK_CURRENT_DURATION_CUBE_HANDLER = 1000;
	public static final int CONTINUOUS_CURRENT_LIMIT_CUBE_HANDLER = 40;

	/**
	 * The 100% voltage that is used as a base calculation for all PercentOutputs
	 */
	public static final double VOLTAGE_COMPENSATION_LEVEL_CUBE_HANDLER = 12;

	/**
	 * The neutral mode of the cube handler (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE_CUBE_HANDLER = NeutralMode.Coast;

	/**
	 * The default timeout of the cube handler functions in ms
	 */
	public static final int DEFAULT_TIMEOUT = 0;
	
	private CubeHandler() {
		
		if(EnabledSubsystems.CUBE_HANDLER_ENABLED) {
		
			cubeHandlerTalon = CTRECreator.createMasterTalon(RobotMap.CUBE_HANDLER_TALON);
				
			cubeHandlerTalon.setNeutralMode(NEUTRAL_MODE_CUBE_HANDLER);
			cubeHandlerTalon.setInverted(false);
	
			cubeHandlerTalon.enableVoltageCompensation(true);
			cubeHandlerTalon.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_CUBE_HANDLER, DEFAULT_TIMEOUT);
	
			cubeHandlerTalon.enableCurrentLimit(true);
			cubeHandlerTalon.configPeakCurrentLimit(PEAK_CURRENT_CUBE_HANDLER, DEFAULT_TIMEOUT);
			cubeHandlerTalon.configPeakCurrentDuration(PEAK_CURRENT_DURATION_CUBE_HANDLER, DEFAULT_TIMEOUT);
			cubeHandlerTalon.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_CUBE_HANDLER, DEFAULT_TIMEOUT);
	
			cubeHandlerTalon.configClosedloopRamp(VOLTAGE_RAMP_RATE_CUBE_HANDLER.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			cubeHandlerTalon.configOpenloopRamp(VOLTAGE_RAMP_RATE_CUBE_HANDLER.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
	
		}
		
		smartDashboardInit();
	}

	public static CubeHandler getInstance() {
		if (singleton == null)
			init();
		return singleton;
	}
	
	public synchronized static void init() {
		if (singleton == null) {
			singleton = new CubeHandler();
		}
	}
	
	/**
	 * @return The current of the cube handler talon
	 */
	public double getCurrent() {
		if (cubeHandlerTalon != null)
			return cubeHandlerTalon.getOutputCurrent();
		return 0;
	}
	
	/**
	 * @param percent velocity
	 */
	public void setMotorSpeedPercent(double percent) {
		if (cubeHandlerTalon != null) {
			cubeHandlerTalon.set(ControlMode.PercentOutput, percent);
		}
	}
	
	/**
	 * What is put on SmartDashboard when it's initialized
	 */
	public void smartDashboardInit() {
		if (EnabledSubsystems.CUBE_HANDLER_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Voltage Ramp Rate Cube Handler Seconds: ",
					VOLTAGE_RAMP_RATE_CUBE_HANDLER.get(Time.Unit.SECOND));
			SmartDashboard.putNumber("Cube Handler Vel Percent: ", VEL_PERCENT_CUBE_HANDLER);
		}
	}
	
	@Override
	public void smartDashboardInfo() {
		if (EnabledSubsystems.CUBE_HANDLER_SMARTDASHBOARD_BASIC_ENABLED) {
			SmartDashboard.putNumber("Cube Handler Current: ", getCurrent());
		}
		if (EnabledSubsystems.CUBE_HANDLER_SMARTDASHBOARD_DEBUG_ENABLED) {
			VOLTAGE_RAMP_RATE_CUBE_HANDLER = new Time(
					SmartDashboard.getNumber("Voltage Ramp Rate Cube Handler Seconds: ",
							VOLTAGE_RAMP_RATE_CUBE_HANDLER.get(Time.Unit.SECOND)), Time.Unit.SECOND);
			
			VEL_PERCENT_CUBE_HANDLER = SmartDashboard.getNumber("Cube Handler Vel Percent: ", VEL_PERCENT_CUBE_HANDLER);
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
