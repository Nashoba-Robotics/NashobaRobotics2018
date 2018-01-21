package edu.nr.robotics.subsystems.cubeHandler;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.sensorhistory.TalonEncoderCubeHandler;
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
	private TalonEncoderCubeHandler cubeHandlerEncoder;
	
	/**
	 * The encoder ticks per inch moved on the cube handler
	 */
	public static final double ENC_TICK_PER_INCH_CUBE_HANDLER = 0; //TODO: Find ENC_TICK_PER_INCH_CUBE_HANDLER
	
	/**
	 * The max speed of the cube handler
	 */
	public static final Speed MAX_SPEED_CUBE_HANDLER = Speed.ZERO; // TODO: Find MAX_SPEED_CUBE_HANDLER
	
	/**
	 * The minimum voltage needed to move the cube handler
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_CUBE_HANDLER = 0; // TODO: Find CubeHandler voltage velocity curve

	/**
	 * The slope of voltage over velocity in feet per second
	 */
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_CUBE_HANDLER = 0;
	
	/**
	 * The voltage ramp rate of the cube handler. Voltage ramp rate is time it takes
	 * to go from 0V to 12V
	 */
	public static Time VOLTAGE_RAMP_RATE_CUBE_HANDLER = Time.ZERO; // TODO: Test for cube handler voltage ramp rate

	/**
	 * Velocity PID values for the cube handler
	 */
	public static double P_VEL_CUBE_HANDLER = 0; // TODO: Find cube handler velocity PID values
	public static double I_VEL_CUBE_HANDLER = 0;
	public static double D_VEL_CUBE_HANDLER = 0;

	/**
	 * The default velocity percent for the cube handler
	 */
	public static double VEL_PERCENT_CUBE_HANDLER = 0;//TODO: Find cube handler velocity percent
	
	/**
	 * The current values of the cube handler
	 */
	public static final int PEAK_CURRENT_CUBE_HANDLER = 0;// TODO: Find PEAK_CURRENT_CUBE_HANDLER
	public static final int PEAK_CURRENT_DURATION_CUBE_HANDLER = 0; // TODO: Find PEAK_CURRENT_DURATION_CUBE_HANDLER
	public static final int CONTINUOUS_CURRENT_LIMIT_CUBE_HANDLER = 0; // TODO: Find CONTINUOUS_CURRENT_LIMIT_CUBE_HANDLER

	/**
	 * The rate of velocity measurements on the cube handler encoder
	 */
	public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_CUBE_HANDLER = VelocityMeasPeriod.Period_10Ms;

	/**
	 * The number of measurements the cube handler encoder averages to return a value
	 */
	public static final double VELOCITY_MEASUREMENT_WINDOW_CUBE_HANDLER = 32;

	/**
	 * The 100% voltage that is used as a base calculation for all PercentOutputs
	 */
	public static final double VOLTAGE_COMPENSATION_LEVEL_CUBE_HANDLER = 12;

	/**
	 * The neutral mode of the cube handler (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE_CUBE_HANDLER = NeutralMode.Coast;

	/**
	 * The PID type of the cube handler. 0 = Primary, 1 = Cascade
	 */
	public static final int PID_TYPE = 0;

	/**
	 * The default timeout of the cube handler functions in ms
	 */
	public static final int DEFAULT_TIMEOUT = 0;

	/**
	 * The PID slot numbers
	 */
	public static final int VEL_SLOT = 0;

	private Speed velSetpoint = Speed.ZERO;

	
private CubeHandler() {
		
		if(EnabledSubsystems.CUBE_HANDLER_ENABLED) {
		
			cubeHandlerTalon = CTRECreator.createMasterTalon(RobotMap.CUBE_HANDLER_TALON);
	
			if (EnabledSubsystems.CUBE_HANDLER_DUMB_ENABLED) {
				cubeHandlerTalon.set(ControlMode.PercentOutput, 0);
			} else {
				cubeHandlerTalon.set(ControlMode.Velocity, 0);
			}
	
			cubeHandlerTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			cubeHandlerTalon.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
			cubeHandlerTalon.config_kP(VEL_SLOT, P_VEL_CUBE_HANDLER, DEFAULT_TIMEOUT);
			cubeHandlerTalon.config_kI(VEL_SLOT, I_VEL_CUBE_HANDLER, DEFAULT_TIMEOUT);
			cubeHandlerTalon.config_kD(VEL_SLOT, D_VEL_CUBE_HANDLER, DEFAULT_TIMEOUT);
			cubeHandlerTalon.setNeutralMode(NEUTRAL_MODE_CUBE_HANDLER);
			cubeHandlerTalon.setInverted(false);
			cubeHandlerTalon.setSensorPhase(false);
	
			cubeHandlerTalon.enableVoltageCompensation(true);
			cubeHandlerTalon.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL_CUBE_HANDLER, DEFAULT_TIMEOUT);
	
			cubeHandlerTalon.enableCurrentLimit(true);
			cubeHandlerTalon.configPeakCurrentLimit(PEAK_CURRENT_CUBE_HANDLER, DEFAULT_TIMEOUT);
			cubeHandlerTalon.configPeakCurrentDuration(PEAK_CURRENT_DURATION_CUBE_HANDLER, DEFAULT_TIMEOUT);
			cubeHandlerTalon.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT_CUBE_HANDLER, DEFAULT_TIMEOUT);
	
			cubeHandlerTalon.configClosedloopRamp(VOLTAGE_RAMP_RATE_CUBE_HANDLER.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			cubeHandlerTalon.configOpenloopRamp(VOLTAGE_RAMP_RATE_CUBE_HANDLER.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
	
			cubeHandlerEncoder = new TalonEncoderCubeHandler(cubeHandlerTalon);
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
	 * @return The current velocity of the cube handler encoders
	 */
	public Speed getVelocity() {
		if (cubeHandlerTalon != null)
			return new Speed(cubeHandlerTalon.getSelectedSensorVelocity(PID_TYPE), Distance.Unit.MAGNETIC_ENCODER_TICK_CUBE_HANDLER,
				Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	/**
	 * Gets the historical velocity of the cube handler talon
	 * 
	 * @param timePassed
	 *            How long ago to look
	 * @return old velocity of the cube handler talon
	 */
	public Speed getHistoricalVelocity(Time timePassed) {
		if (cubeHandlerTalon != null)
			return new Speed(cubeHandlerEncoder.getVelocity(timePassed));
		return Speed.ZERO;
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
		setMotorSpeed(MAX_SPEED_CUBE_HANDLER.mul(percent));
	}

	/**
	 * @param speed to set motor in
	 */
	public void setMotorSpeed(Speed speed) {

		if (cubeHandlerTalon != null) {

			velSetpoint = speed;
			cubeHandlerTalon.config_kF(VEL_SLOT,
					((VOLTAGE_PERCENT_VELOCITY_SLOPE_CUBE_HANDLER * velSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
							+ MIN_MOVE_VOLTAGE_PERCENT_CUBE_HANDLER) * 1023.0)
							/ velSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_CUBE_HANDLER,
									Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
	
			if (cubeHandlerTalon.getControlMode() == ControlMode.PercentOutput) {
				cubeHandlerTalon.set(cubeHandlerTalon.getControlMode(), velSetpoint.div(MAX_SPEED_CUBE_HANDLER));
			} else {
				cubeHandlerTalon.set(cubeHandlerTalon.getControlMode(),
						velSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_CUBE_HANDLER, Time.Unit.HUNDRED_MILLISECOND));
			}
		}
	}
	
	/**
	 * What is put on SmartDashboard when it's initialized
	 */
	public void smartDashboardInit() {
		if (EnabledSubsystems.CUBE_HANDLER_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putNumber("Voltage Ramp Rate Cube Handler Seconds: ",
					VOLTAGE_RAMP_RATE_CUBE_HANDLER.get(Time.Unit.SECOND));
			SmartDashboard.putNumber("P Vel Cube Handler: ", P_VEL_CUBE_HANDLER);
			SmartDashboard.putNumber("I Vel Cube Handler: ", I_VEL_CUBE_HANDLER);
			SmartDashboard.putNumber("D Vel Cube Handler: ", D_VEL_CUBE_HANDLER);
			SmartDashboard.putNumber("Cube Handler Vel Percent: ", VEL_PERCENT_CUBE_HANDLER);
		}
	}
	
	@Override
	public void smartDashboardInfo() {
		if (EnabledSubsystems.CUBE_HANDLER_SMARTDASHBOARD_BASIC_ENABLED) {
			SmartDashboard.putNumber("Cube Handler Current: ", getCurrent());
			SmartDashboard.putNumber("Cube Handler Velocity vs Set Velocity: ", velSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		}
		if (EnabledSubsystems.CUBE_HANDLER_SMARTDASHBOARD_DEBUG_ENABLED) {
			VOLTAGE_RAMP_RATE_CUBE_HANDLER = new Time(
					SmartDashboard.getNumber("Voltage Ramp Rate Cube Handler Seconds: ",
							VOLTAGE_RAMP_RATE_CUBE_HANDLER.get(Time.Unit.SECOND)), Time.Unit.SECOND);
			P_VEL_CUBE_HANDLER = SmartDashboard.getNumber("P Vel Cube Handler: ", P_VEL_CUBE_HANDLER);
			I_VEL_CUBE_HANDLER = SmartDashboard.getNumber("I Vel Cube Handler: ", I_VEL_CUBE_HANDLER);
			D_VEL_CUBE_HANDLER = SmartDashboard.getNumber("D Vel Cube Handler: ", D_VEL_CUBE_HANDLER);
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
