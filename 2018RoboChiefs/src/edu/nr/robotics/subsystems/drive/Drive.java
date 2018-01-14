package edu.nr.robotics.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.NRMath;
import edu.nr.lib.gyro.NavX;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.driving.DriveTypeCalculations;
import edu.nr.lib.gyro.Gyro;
import edu.nr.lib.gyro.Pigeon;
import edu.nr.lib.gyro.Gyro.ChosenGyro;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.interfaces.TriplePIDOutput;
import edu.nr.lib.interfaces.TriplePIDSource;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerHDriveMain;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotorHDrive;
import edu.nr.lib.motionprofiling.OneDimensionalTrajectoryRamped;
import edu.nr.lib.sensorhistory.TalonEncoder;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.AngularSpeed;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Jerk;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.lib.units.Distance.Unit;
import edu.nr.robotics.OI;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends NRSubsystem implements TriplePIDOutput, TriplePIDSource {

	private static Drive singleton;
	
	private TalonSRX leftDrive, rightDrive, leftDriveFollow, rightDriveFollow, hDrive, hDriveFollow, pigeonTalon;
	private TalonEncoder leftEncoder, rightEncoder, hEncoder;

	/**
	 * The max drive current in amperes
	 */
	public static final double MAX_DRIVE_CURRENT = 0; //TODO: Find real value of max drive current
	
	/**
	 * The diameter of the drive wheels in inches
	 */
	public static final double WHEEL_DIAMETER_INCHES = 0; //TODO: Find real drive wheel diameter
	
	/**
	 * The diameter of the H-drive wheel in inches
	 */
	public static final double WHEEL_DIAMETER_INCHES_H = 0;//TODO: Find real drive H-wheel diameter
	
	/**
	 * The diameter of the drive wheels
	 */
	public static final Distance WHEEL_DIAMETER = new Distance(WHEEL_DIAMETER_INCHES, Distance.Unit.INCH);

	/**
	 * The maximum speed of the drive base
	 */
	public static final Speed MAX_SPEED = Speed.ZERO; //TODO: Find real drive max speed
	public static final Speed MAX_SPEED_H = Speed.ZERO; //TODO: Find real drive max speed h
	
	/**
	 * The maximum acceleration of the drive base
	 */
	public static final Acceleration MAX_ACCELERATION = Acceleration.ZERO; //TODO: Find real drive max acceleration
	public static final Acceleration MAX_ACCELERATION_H = Acceleration.ZERO; //TODO: Find real drive max acceleration h
	
	/**
	 * Ticks per revolution of the drive encoders
	 */
	public static final int TICKS_PER_REV = 0; //TODO: Get real ticks per rev of the drive encoders
	
	/**
	 * The drive voltage-velocity curve slopes
	 */
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0; //TODO: Find drive voltage vs velocity curve
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_H = 0;
	
	/**
	 * Voltage percentage at which robot just starts moving
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_H = 0; //This si 0 to 1 number
	
	/**
	 * The CANTalon PID values for velocity
	 */
	public static double P_LEFT = 0; //TODO: Find left PID values
	public static double I_LEFT = 0;
	public static double D_LEFT = 0;
		
	public static double P_RIGHT = 0; //TODO: Find right PID values
	public static double I_RIGHT = 0;
	public static double D_RIGHT = 0;
	
	public static double P_H = 0; //TODO: Find h PID values
	public static double I_H = 0;
	public static double D_H = 0;
	
	/**
	 * Type of PID. 0 = primary. 1 = cascade
	 */
	public static final int PID_TYPE = 0;
	
	/**
	 * Acceptable position error in motion profiling
	 */
	public static final Distance PROFILE_POSITION_THRESHOLD = new Distance(0, Distance.Unit.INCH); //TODO: Find Drive profile position threshold
	
	/**
	 * Time stopped before motion profiling ends
	 */
	public static final Time PROFILE_TIME_THRESHOLD = Time.ZERO; //TODO: Find Drive profile time threshold
	
	public static final double ACCEL_PERCENT = 0; //TODO: Find optimal drive profiling acceleration percent
	
	public static final double PROFILE_DRIVE_PERCENT = 0; //TODO: Find Drive profile percent
	
	/**
	 * No timeout for talon configuration functions
	 */
	public static final int NO_TIMEOUT = 0;
	
	/**
	 * Used to prevent magic numbers from showing up in code
	 */
	public static final int SLOT_0 = 0;
	public static final int SLOT_1 = 1;
	
	/**
	 * Current ratings based on MAXI Circuit Breaker Model MX5
	 */
	private static final int PEAK_CURRENT = 80; //In amps
	private static final int PEAK_CURRENT_DURATION = 1000; //In milliseconds
	private static final int CONTINUOUS_CURRENT_LIMIT = 40; //In amps
	
	/**
	 * Voltage level considered 100% for calculation
	 */
	private static final int VOLTAGE_COMPENSATION_LEVEL = 12; //In volts
	
	/**
	 * Tracking of the drive motor setpoints
	 */
	private Speed leftMotorSetpoint = Speed.ZERO;
	private Speed rightMotorSetpoint = Speed.ZERO;
	private Speed hMotorSetpoint = Speed.ZERO;
	private double oldTurn;

	/**
	 * Possible drive mode selections
	 */
	public static enum DriveMode {
		arcadeDrive, tankDrive, cheesyDrive
	}
	
	/**
	 * Default neutral mode (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
	
	/**
	 * @return max drive speed in current gearing
	 */
	public Speed currentMaxSpeed() {
		return MAX_SPEED;
	}
	
	/**
	 * @return max drive speed h in current gearing
	 */
	public Speed currentMaxSpeedH() {
		return MAX_SPEED_H;
	}
	
	/**
	 * 1D Profiling kVAPID_theta loop constants
	 */
	public static double kVOneD = 1 / MAX_SPEED.get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND);
	public static double kAOneD = 0;
	public static double kPOneD = 0;
	public static double kIOneD = 0;
	public static double kDOneD = 0;
	public static double kP_thetaOneD = 0;
	
	//TODO: Find These
	public static double kVOneDH = 1 / MAX_SPEED.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND);
	public static double kAOneDH = 0;
	public static double kPOneDH = 0;
	public static double kIOneDH = 0;
	public static double kDOneDH = 0; 
	
	/**
	 * Motion profiling parameters (x is front-back, y is left-right)
	 */
	public static Distance xProfile;
	public static Distance yProfile;
	public static Angle endAngle;
	public static double drivePercent = 0;
	
	private OneDimensionalMotionProfilerTwoMotorHDrive oneDProfilerTwoMotorH;
	
	private OneDimensionalMotionProfilerHDriveMain oneDProfilerOneMotorH;
		
	private Drive() {
		if (EnabledSubsystems.DRIVE_ENABLED) {

			leftDrive = CTRECreator.createMasterTalon(RobotMap.LEFT_DRIVE);
			rightDrive = CTRECreator.createMasterTalon(RobotMap.RIGHT_DRIVE);
			hDrive = CTRECreator.createMasterTalon(RobotMap.H_DRIVE);
			pigeonTalon = CTRECreator.createMasterTalon(0);//TODO: find real pigeon talon
			
			leftDriveFollow = CTRECreator.createFollowerTalon(RobotMap.LEFT_DRIVE_FOLLOW, leftDrive.getDeviceID());
			rightDriveFollow = CTRECreator.createFollowerTalon(RobotMap.RIGHT_DRIVE_FOLLOW, rightDrive.getDeviceID());
			hDriveFollow = CTRECreator.createFollowerTalon(RobotMap.H_DRIVE_FOLLOW, hDrive.getDeviceID());
			
			if (EnabledSubsystems.DRIVE_DUMB_ENABLED) {
				leftDrive.set(ControlMode.PercentOutput, 0);
				rightDrive.set(ControlMode.PercentOutput, 0);
				hDrive.set(ControlMode.PercentOutput, 0);
			} else {
				leftDrive.set(ControlMode.Velocity, 0);
				rightDrive.set(ControlMode.Velocity, 0);
				hDrive.set(ControlMode.Velocity, 0);
			}
			
			leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, NO_TIMEOUT);
			leftDrive.config_kF(SLOT_0, 0, NO_TIMEOUT);
			leftDrive.config_kP(SLOT_0, P_LEFT, NO_TIMEOUT);
			leftDrive.config_kI(SLOT_0, I_LEFT, NO_TIMEOUT);
			leftDrive.config_kD(SLOT_0, D_LEFT, NO_TIMEOUT);
			leftDrive.setNeutralMode(NEUTRAL_MODE);
			leftDrive.setInverted(false);
			leftDriveFollow.setInverted(false);
			leftDrive.setSensorPhase(false);
			leftDriveFollow.setSensorPhase(false);
			
			leftDrive.enableVoltageCompensation(true);
			leftDrive.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL, NO_TIMEOUT);
			
			leftDrive.enableCurrentLimit(true);
			leftDrive.configPeakCurrentLimit(PEAK_CURRENT, NO_TIMEOUT);
			leftDrive.configPeakCurrentDuration(PEAK_CURRENT_DURATION, NO_TIMEOUT);
			leftDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, NO_TIMEOUT);
			
			leftEncoder = new TalonEncoder(leftDrive);
			
			leftDriveFollow.setNeutralMode(NEUTRAL_MODE);
			
			rightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, NO_TIMEOUT);
			rightDrive.config_kF(SLOT_0, 0, NO_TIMEOUT);
			rightDrive.config_kP(SLOT_0, P_RIGHT, NO_TIMEOUT);
			rightDrive.config_kI(SLOT_0, I_RIGHT, NO_TIMEOUT);
			rightDrive.config_kD(SLOT_0, D_RIGHT, NO_TIMEOUT);
			rightDrive.setNeutralMode(NEUTRAL_MODE);			
			rightDrive.setInverted(false);
			rightDrive.setInverted(false);
			rightDrive.setSensorPhase(false);
			rightDrive.setSensorPhase(false);
			
			rightDrive.enableVoltageCompensation(true);
			rightDrive.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL, NO_TIMEOUT);
			
			rightDrive.enableCurrentLimit(true);
			rightDrive.configPeakCurrentLimit(PEAK_CURRENT, NO_TIMEOUT);
			rightDrive.configPeakCurrentDuration(PEAK_CURRENT_DURATION, NO_TIMEOUT);
			rightDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, NO_TIMEOUT);
			
			rightEncoder = new TalonEncoder(rightDrive);
			
			rightDriveFollow.setNeutralMode(NEUTRAL_MODE);
			
			hDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, NO_TIMEOUT);
			hDrive.config_kF(SLOT_0, 0, NO_TIMEOUT);
			hDrive.config_kP(SLOT_0, P_LEFT, NO_TIMEOUT);
			hDrive.config_kI(SLOT_0, I_LEFT, NO_TIMEOUT);
			hDrive.config_kD(SLOT_0, D_LEFT, NO_TIMEOUT);
			hDrive.setNeutralMode(NEUTRAL_MODE);
			hDrive.setInverted(false);
			hDriveFollow.setInverted(false);
			hDrive.setSensorPhase(false);
			hDriveFollow.setSensorPhase(false);
			
			hDrive.enableVoltageCompensation(true);
			hDrive.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL, NO_TIMEOUT);
			
			hDrive.enableCurrentLimit(true);
			hDrive.configPeakCurrentLimit(PEAK_CURRENT, NO_TIMEOUT);
			hDrive.configPeakCurrentDuration(PEAK_CURRENT_DURATION, NO_TIMEOUT);
			hDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, NO_TIMEOUT);
			
			hEncoder = new TalonEncoder(hDrive);
			
			hDriveFollow.setNeutralMode(NEUTRAL_MODE);
			
			smartDashboardInit();
	
			CheesyDriveCalculationConstants.createDriveTypeCalculations();
			
		}
	}
	
	public static Drive getInstance() {
		if (singleton == null)
			init();
		return singleton;
	}
	
	public synchronized static void init() {
		if (singleton == null) {
			singleton = new Drive();
			singleton.setJoystickCommand(new DriveJoystickCommand());
		}
	}

	public TalonSRX getPigeonTalon() {
		return pigeonTalon;
	}
	
	/**
	 * Uses 254's CheesyDrive to drive
	 * 
	 * @param move 
	 * 				The speed, from -1 to 1 (inclusive), that the robot should go
	 *             	at. 1 is max forward, 0 is stopped, -1 is max backward
	 * @param turn 
	 * 				The speed, from -1 to 1 (inclusive), that the robot should
	 *            	turn at. 1 is max right, 0 is stopped, -1 is max left
	 */
	public void cheesyDrive(double move, double turn, double strafe) {
		double[] cheesyMotorPercents = new double[2];
		cheesyMotorPercents = DriveTypeCalculations.cheesyDrive(move, turn, oldTurn, false);
		
		oldTurn = turn;
		
		tankDrive(cheesyMotorPercents[0], cheesyMotorPercents[1], strafe);		
	}
	
	/**
	 * An arcade drive
	 * 
	 * @param move The speed, from -1 to 1 (inclusive), that the robot should go
	 *             	at. 1 is max forward, 0 is stopped, -1 is max backward
	 * @param turn The speed, from -1 to 1 (inclusive), that the robot should
	 *            	turn at. 1 is max right, 0 is stopped, -1 is max left
	 * @param strafeRaw The speed, from -1 to 1 (inclusive), that the robot should
	 * 				strafe at. 1 is max right, 0 is stopped, -1 is max left
	 */
	public void arcadeDrive(double move, double turn, double strafeRaw) {
		
		double[] motorPercents = new double[2];
		motorPercents = DriveTypeCalculations.arcadeDrive(move, turn);
		
		double strafe = NRMath.limit(strafeRaw);
		tankDrive(motorPercents[0], motorPercents[1], strafe);
	}
	
	public void tankDrive(double left, double right, double strafe) {
		setMotorSpeedInPercent(left, right, strafe);	
	}
	
	/**
	 * @return Current position of the left talon
	 */
	public Distance getLeftPosition() {
		if(leftDrive != null) {
			return new Distance(leftDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK);
		}		
		else {
			return Distance.ZERO;
		}
	}
	
	/**
	 * @return Current position of the right talon
	 */
	public Distance getRightPosition() {
		if(rightDrive != null) {
			return new Distance(rightDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK);
		}
		else {
			return Distance.ZERO;
		}
	}
	
	/**
	 * @return Current position of the h talon
	 */
	public Distance getHPosition() {
		if(hDrive != null){
			return new Distance(rightDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK);
		}
		else{
			return Distance.ZERO;
		}
	}
	
	/**
	 * Gets the historical position of the left talon
	 * 
	 * @param deltaTime
	 *            How long ago to look
	 * @return old position of the left talon
	 */
	public Distance getHistoricalLeftPosition(Time deltaTime) {
		if (leftEncoder != null)
			return leftEncoder.getPosition(deltaTime);
		return Distance.ZERO;
	}

	/**
	 * Gets the historical position of the right talon
	 * 
	 * @param deltaTime
	 *            How long ago to look
	 * @return old position of the right talon
	 */
	public Distance getHistoricalRightPosition(Time deltaTime) {
		if (rightEncoder != null)
			return rightEncoder.getPosition(deltaTime);
		return Distance.ZERO;
	}
	
	/**
	 * Gets the historical position of the h talon
	 * 
	 * @param deltaTime
	 * 				How long ago to look
	 * @return old position of the h talon
	 */
	public Distance getHistoricalHPosition(Time deltaTime) {
		if (hEncoder != null)
			return hEncoder.getPosition(deltaTime);
		return Distance.ZERO;
	}
	
	public void setMotorSpeedInPercent(double left, double right, double strafe) {
		setMotorSpeed(currentMaxSpeed().mul(left), currentMaxSpeed().mul(right), currentMaxSpeedH().mul(strafe));
	}
	
	public void setMotorSpeed(Speed left, Speed right, Speed strafe) {
		if (leftDrive != null && rightDrive != null && hDrive != null) {
			
			leftMotorSetpoint = left;
			rightMotorSetpoint = right;
			hMotorSetpoint = strafe;
			
			leftDrive.config_kF(SLOT_0, ((VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT * leftMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_LEFT) * 1023.0) / leftMotorSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND), NO_TIMEOUT);
			rightDrive.config_kF(SLOT_0, ((VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT * rightMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_RIGHT) * 1023.0) / rightMotorSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND), NO_TIMEOUT);
			hDrive.config_kF(SLOT_0, ((VOLTAGE_PERCENT_VELOCITY_SLOPE_H * hMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_H) * 1023.0) / hMotorSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND), NO_TIMEOUT);
			
			if (leftDrive.getControlMode() == ControlMode.PercentOutput) {
				leftDrive.set(leftDrive.getControlMode(), leftMotorSetpoint.div(currentMaxSpeed()));
			} else {
				leftDrive.set(leftDrive.getControlMode(), leftMotorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND));
			}
			if (rightDrive.getControlMode() == ControlMode.PercentOutput) {
				rightDrive.set(rightDrive.getControlMode(), rightMotorSetpoint.div(currentMaxSpeed()));
			} else {
				rightDrive.set(rightDrive.getControlMode(), rightMotorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND));
			}
			if (hDrive.getControlMode() == ControlMode.PercentOutput) {
				hDrive.set(hDrive.getControlMode(), hMotorSetpoint.div(currentMaxSpeedH()));
			} else {
				hDrive.set(hDrive.getControlMode(), hMotorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND));
			}	
		}
	}
	
	/**
	 * @return Current velocity of the left talon
	 */
	public Speed getLeftVelocity() {
		if (leftDrive != null)
			return new Speed(leftDrive.getSelectedSensorVelocity(SLOT_0), Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	/**
	 * @return Current velocity of the right talon
	 */
	public Speed getRightVelocity() {
		if(rightDrive != null)
			return new Speed(rightDrive.getSelectedSensorVelocity(SLOT_0), Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	/**
	 * @return Current velocity of the h talon
	 */
	public Speed getHVelocity() {
		if(hDrive != null) 
			return new Speed(hDrive.getSelectedSensorVelocity(SLOT_0), Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	/**
	 * Historical velocity of the left talon
	 * 
	 * @param deltaTime
	 * 			How long ago to look
	 * @return Velocity of left talon
	 */
	public Speed getHistoricalLeftVelocity(Time deltaTime) {
		if (leftEncoder != null)
			return new Speed(leftEncoder.getVelocity(deltaTime));
		return Speed.ZERO;
	}
	
	/**
	 * Historical velocity of the right talon
	 * 
	 * @param deltaTime
	 * 			How long ago to look
	 * @return Velocity of right talon
	 */
	public Speed getHistoricalRightVelocity(Time deltaTime) {
		if (rightEncoder != null)
			return new Speed(rightEncoder.getVelocity(deltaTime));
		return Speed.ZERO;
	}
	
	/**
	 * Historical velocity of the h talon
	 * 
	 * @param deltaTime
	 * 			How long ago to look
	 * @return Velocity of h talon
	 */
	public Speed getHistoricalHVelocity(Time deltaTime) {
		if (hEncoder != null)
			return new Speed(hEncoder.getVelocity(deltaTime));
		return Speed.ZERO;
	}
	
	public void startDumbDrive() {
		if (leftDrive != null && rightDrive != null && hDrive != null) {
			if (rightDrive.getControlMode() != ControlMode.PercentOutput) {
				rightDrive.set(ControlMode.PercentOutput, getRightVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND));
			}
			if (leftDrive.getControlMode() != ControlMode.PercentOutput) {
				leftDrive.set(ControlMode.PercentOutput, getLeftVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND));
			}
			if(hDrive.getControlMode() != ControlMode.PercentOutput) {
				hDrive.set(ControlMode.PercentOutput, getHVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND));
			}
		}
	}
	
	public void endDumbDrive() {
		if (leftDrive != null && rightDrive != null && hDrive != null) {
			if (rightDrive.getControlMode() != ControlMode.Velocity) {
				rightDrive.set(ControlMode.Velocity, 0);
			}
			if (leftDrive.getControlMode() != ControlMode.Velocity) {
				leftDrive.set(ControlMode.Velocity, 0);
			}
			if(hDrive.getControlMode() != ControlMode.Velocity) {
				hDrive.set(ControlMode.Velocity, 0);
			}
		}
	}
	
	@Override
	public void periodic() {
	}
	
	/**
	 * What should be displayed when Drive class initialized
	 */
	private void smartDashboardInit() {
		if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
		}
	}
	
	@Override
	public void smartDashboardInfo() {
		if (leftDrive != null && rightDrive != null) {
			if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_BASIC_ENABLED) {
				
				SmartDashboard.putString("Drive Current", getLeftCurrent() + " : " + getRightCurrent() + " : " + getHCurrent());
				
				if (Gyro.chosenGyro.equals(ChosenGyro.NavX)) {
					SmartDashboard.putNumber("Gyro Yaw", NavX.getInstance().getYaw().get(Angle.Unit.DEGREE));
				} else {
					SmartDashboard.putNumber("Gyro Yaw", Pigeon.getPigeon(getInstance().getPigeonTalon()).getYaw().get(Angle.Unit.DEGREE));
				}
				
				SmartDashboard.putString("Drive Left Velocity: ", getLeftVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
				SmartDashboard.putString("Drive Right Velocity: ", getRightVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
				SmartDashboard.putString("Drive H Velocity: ", getHVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + hMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
				
			}
			if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
				
				SmartDashboard.putNumber("Drive Left Percent", leftMotorSetpoint.div(currentMaxSpeed()));
				SmartDashboard.putNumber("Drive Right Percent", rightMotorSetpoint.div(currentMaxSpeed()));
				SmartDashboard.putNumber("Drive H Percent", hMotorSetpoint.div(currentMaxSpeed()));
				
				SmartDashboard.putData(this);
				SmartDashboard.putString("Drive Voltage",
						leftDrive.getMotorOutputVoltage() + " : " + rightDrive.getMotorOutputVoltage() + " : " + hDrive.getMotorOutputVoltage());
				SmartDashboard.putNumber("Drive Left Position", getLeftPosition().get(Distance.Unit.INCH));
				SmartDashboard.putNumber("Drive Right Position", getRightPosition().get(Distance.Unit.INCH));
				SmartDashboard.putNumber("Drive H Position", getHPosition().get(Distance.Unit.INCH));
				
				leftDrive.config_kP(SLOT_0, SmartDashboard.getNumber("Left P Value: ", P_LEFT), NO_TIMEOUT);
				leftDrive.config_kI(SLOT_0, SmartDashboard.getNumber("Left I Value: ", I_LEFT), NO_TIMEOUT);
				leftDrive.config_kD(SLOT_0, SmartDashboard.getNumber("Left D Value: ", D_LEFT), NO_TIMEOUT);
				
				rightDrive.config_kP(SLOT_0, SmartDashboard.getNumber("Right P Value: ", P_RIGHT), NO_TIMEOUT);
				rightDrive.config_kI(SLOT_0, SmartDashboard.getNumber("Right I Value: ", I_RIGHT), NO_TIMEOUT);
				rightDrive.config_kD(SLOT_0, SmartDashboard.getNumber("Right D Value: ", D_RIGHT), NO_TIMEOUT);
				
				hDrive.config_kP(SLOT_0, SmartDashboard.getNumber("H P Value: ", P_H), NO_TIMEOUT);
				hDrive.config_kI(SLOT_0, SmartDashboard.getNumber("H I Value: ", I_H), NO_TIMEOUT);
				hDrive.config_kD(SLOT_0, SmartDashboard.getNumber("H D Value: ", D_H), NO_TIMEOUT);
	
				P_LEFT = SmartDashboard.getNumber("Left P Value: ", P_LEFT);
				I_LEFT = SmartDashboard.getNumber("Left I Value: ", I_LEFT);
				D_LEFT = SmartDashboard.getNumber("Left D Value: ", D_LEFT);
				
				P_RIGHT = SmartDashboard.getNumber("Right P Value: ", P_RIGHT);
				I_RIGHT = SmartDashboard.getNumber("Right I Value: ", I_RIGHT);
				D_RIGHT = SmartDashboard.getNumber("Right D Value: ", D_RIGHT);
				
				P_H = SmartDashboard.getNumber("H P Value: ", P_H);
				I_H = SmartDashboard.getNumber("H I Value: ", I_H);
				D_H = SmartDashboard.getNumber("H D Value: ", D_H);
				
				kVOneD = SmartDashboard.getNumber("kVOneD Value: ", kVOneD);
				kAOneD = SmartDashboard.getNumber("kAOneD Value: ", kAOneD);
				kPOneD = SmartDashboard.getNumber("kPOneD Value: ", kPOneD);
				kIOneD = SmartDashboard.getNumber("kIOneD Value: ", kIOneD);
				kDOneD = SmartDashboard.getNumber("kDOneD Value: ", kDOneD);
				kP_thetaOneD = SmartDashboard.getNumber("kP_thetaOneD Value: ", kP_thetaOneD);
				
				
				kVOneDH = SmartDashboard.getNumber("kVOneDH Value: ", kVOneDH);
				kAOneDH = SmartDashboard.getNumber("kAOneDH Value: ", kAOneDH);
				kPOneDH = SmartDashboard.getNumber("kPOneDH Value: ", kPOneDH);
				kIOneDH = SmartDashboard.getNumber("kIOneDH Value: ", kIOneDH);
				kDOneDH = SmartDashboard.getNumber("kDOneDH Value: ", kDOneDH);
				
				xProfile = new Distance(SmartDashboard.getNumber("X Profile Feet", 0), Distance.Unit.FOOT);
				yProfile = new Distance(SmartDashboard.getNumber("Y Profile Feet", 0), Distance.Unit.FOOT);
				endAngle = new Angle(SmartDashboard.getNumber("End Degree", 0), Angle.Unit.DEGREE);
				drivePercent = SmartDashboard.getNumber("Drive Percent", 0);
			}
		}
	}
	
	@Override
	public void disable() {
		setMotorSpeedInPercent(0, 0, 0);
	}
	
	public void enableOneDProfilerTwoMotorH(Distance dist) {
		oneDProfilerTwoMotorH = new OneDimensionalMotionProfilerTwoMotorHDrive(this, this, kVOneD, kAOneD, kPOneD, kIOneD, kDOneD, kP_thetaOneD, kPOneDH, kIOneDH, kDOneDH);
		oneDProfilerTwoMotorH.setTrajectory(new OneDimensionalTrajectoryRamped(dist.get(Distance.Unit.MAGNETIC_ENCODER_TICK), 
				MAX_SPEED.mul(drivePercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND), 
				MAX_ACCELERATION.mul(ACCEL_PERCENT).get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND)));
		oneDProfilerTwoMotorH.enable();
	}
	
	public void enableOneDProfilerOneMotorH(Distance dist) {
		oneDProfilerOneMotorH = new OneDimensionalMotionProfilerHDriveMain(this, this, kVOneDH, kAOneDH, kPOneDH, kIOneDH, kDOneDH, kP_thetaOneD);
		oneDProfilerOneMotorH.setTrajectory(new OneDimensionalTrajectoryRamped(dist.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H), 
				MAX_SPEED_H.mul(drivePercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND), 
				MAX_ACCELERATION_H.mul(ACCEL_PERCENT).get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND)));
		oneDProfilerOneMotorH.enable();	
	}
	
	public void disableOneDProfilerTwoMotorH() {
		oneDProfilerTwoMotorH.disable();
	}
	
	public void disableOneDProfilerOneMotorH() {
		oneDProfilerOneMotorH.disable();
	}
	
	/**
	 * @return Current of the right drive talon
	 */
	public double getRightCurrent() {
		if (rightDrive != null) {
			return rightDrive.getOutputCurrent();
		}
		return 0;
	}
	
	/**
	 * @return Current of the left drive talon
	 */
	public double getLeftCurrent() {
		if (leftDrive != null) {
			return leftDrive.getOutputCurrent();
		}
		return 0;
	}
	
	/**
	 * @return Current of the h drive talon
	 */
	public double getHCurrent() {
		if (hDrive != null) {
			return hDrive.getOutputCurrent();
		}
		return 0;
	}
	
	public void setVoltageRampRate(double rampRate) {
		if(rightDrive != null) {
			rightDrive.configOpenloopRamp(rampRate, NO_TIMEOUT);
		} 
		if(rightDriveFollow != null) {
			rightDriveFollow.configOpenloopRamp(rampRate, NO_TIMEOUT);
		}
		if(leftDrive != null) {
			leftDrive.configOpenloopRamp(rampRate, NO_TIMEOUT);
		}
		if(leftDriveFollow != null) {
			leftDriveFollow.configOpenloopRamp(rampRate, NO_TIMEOUT);
		}
		if(hDrive != null) {
			hDrive.configOpenloopRamp(rampRate, NO_TIMEOUT);
		}
		if(hDriveFollow != null) {
			hDriveFollow.configOpenloopRamp(rampRate, NO_TIMEOUT);
		}
	}
	
	private PIDSourceType type = PIDSourceType.kRate;

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		type = pidSource;
	}
	
	@Override
	public PIDSourceType getPIDSourceType() {
		return type;
	}
	
	@Override
	public double pidGetLeft() {
		if (type == PIDSourceType.kRate) {
			return getInstance().getLeftVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND);
		} else {
			return getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK);
		}
	}
	
	@Override
	public double pidGetRight() {
		if (type == PIDSourceType.kRate) {
			return -getInstance().getRightVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND);
		} else {
			return -getInstance().getRightPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK);
		}
	}
	
	@Override
	public double pidGetH() {
		if (type == PIDSourceType.kRate) {
			return -getInstance().getHVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND);
		} else {
			return -getInstance().getHPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_H);
		}
	}
	
	@Override
	public void pidWrite(double outputLeft, double outputRight, double outputH) {
		setMotorSpeed(currentMaxSpeed().mul(outputLeft),currentMaxSpeed().mul(outputRight), currentMaxSpeedH().mul(outputH));
	}
}
