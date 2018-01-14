package edu.nr.robotics.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.NRMath;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.driving.DriveTypeCalculations;
import edu.nr.lib.gyro.Gyro;
import edu.nr.lib.gyro.Gyro.ChosenGyro;
import edu.nr.lib.gyro.NavX;
import edu.nr.lib.gyro.Pigeon;
import edu.nr.lib.interfaces.TriplePIDOutput;
import edu.nr.lib.interfaces.TriplePIDSource;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerHDriveMain;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotorHDrive;
import edu.nr.lib.motionprofiling.OneDimensionalTrajectoryRamped;
import edu.nr.lib.sensorhistory.TalonEncoder;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.AngularSpeed;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Distance.Unit;
import edu.nr.lib.units.Jerk;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends NRSubsystem implements TriplePIDOutput, TriplePIDSource {
	
	private static Drive singleton;
	
	TalonSRX leftDrive, leftDriveFollow, rightDrive, rightDriveFollow, hDrive, hFollowDrive;
	TalonEncoder leftEncoder, rightEncoder, hEncoder;
	
	public static final double WHEEL_DIAMETER_INCHES = 6;
	public static final double WHEEL_DIAMETER_INCHES_H = 4;
	public static final Distance WHEEL_DIAMETER = new Distance(WHEEL_DIAMETER_INCHES, Distance.Unit.INCH);
	public static final Distance WHEEL_DIAMETER_H = new Distance(WHEEL_DIAMETER_INCHES_H, Distance.Unit.INCH);
	
	//TODO: find all
	public static final Speed MAX_SPEED = new Speed(13.142, Distance.Unit.FOOT, Time.Unit.SECOND);
	public static final Acceleration MAX_ACC = new Acceleration(16, Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND);
	
	public static final Speed MAX_SPEED_H = new Speed(0, Distance.Unit.FOOT, Time.Unit.SECOND);
	public static final Acceleration MAX_ACC_H = new Acceleration(0, Distance.Unit.DRIVE_ROTATION_H, Time.Unit.SECOND, Time.Unit.SECOND);
	
	public static final Distance PROFILE_POSITION_THRESHOLD = new Distance(0.01, Distance.Unit.INCH);
	public static final Time PROFILE_TIME_THRESHOLD = new Time(0.25, Time.Unit.SECOND);
	
	public static final double ACCEL_PERCENT = 0.5;
	
	//TODO: find for all
	public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0.0965; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0.0927; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_H = 0.0;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0.0670;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0.0685;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_H = 0;
	
	public static double ampTimerStart = Timer.getFPGATimestamp();
	public static boolean ampTimerStarted = false;
	public static final double MAX_CURRENT_PERIOD = 0.5;
	
	public static final double MAX_DRIVE_CURRENT = 25; //in amps, maximum current while driving normally
	public static final double ABOVE_MAX_CURRENT_DRIVE_PERCENT = 0.8; //if the max current is reached, it will run at this percent voltage instead
	
	public Speed rightMotorSetpoint = Speed.ZERO;
	public Speed leftMotorSetpoint = Speed.ZERO;
	public Speed hMotorSetpoint = Speed.ZERO;
	public double oldTurn = 0;
	
	//TODO: Tune PID for all
	public static double P_RIGHT = 3.5;
	public static double I_RIGHT = 0;
	public static double D_RIGHT = 35;
	
	public static double P_LEFT = 3.5;
	public static double I_LEFT = 0;
	public static double D_LEFT = 35;
	
	public static double P_H = 0;
	public static double I_H = 0;
	public static double D_H = 0;
	
	public static final double TICKS_PER_REV = 1024; 
	
	//Based on MAXI circuit breaker model
	public static final int PEAK_CURRENT = 80; //In amps
	public static final int PEAK_CURRENT_DURATION = 1000; //In milliseconds
	public static final int CONTINUOUS_CURRENT_LIMIT = 40; //In amps
	
	PIDSourceType type = PIDSourceType.kRate;
	
	public static final int PID_TYPE = 0; //0 = primary, 1 = cascade
	public static final int NO_TIMEOUT = 0;
	public static final int SLOT_0 = 0;
	public static final double VOLTAGE_COMPENSATION_LEVEL = 12; //In volts
	
	public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
	
	public static double kVOneD = 1 / MAX_SPEED.get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND);
	public static double kAOneD = 0.0018;
	public static double kPOneD = 0.0001;
	public static double kIOneD = 0;
	public static double kDOneD = 0;
	public static double kP_thetaOneD = 0.05;
	
	public static double H_kVOneD = 1 / MAX_SPEED_H.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND);
	public static double H_kAOneD = 0.0;
	public static double H_kPOneD = 0;
	public static double H_kIOneD = 0;
	public static double H_kDOneD = 0;
	public static double H_kP_thetaOneD = 0;
	
	public static double HkpOneDAssist = 0;
	public static double HkiOneDAssist  = 0;
	public static double HkdOneDAssist = 0;
	
	public static Distance oneDProfileHMain;
	public static Distance oneDProfileHAssist;
	public static double drivePercent = 0;
	public static double drivePercentH = 0;
	public static Angle angleToTurn = Angle.ZERO;
	
	private OneDimensionalMotionProfilerTwoMotorHDrive oneDProfiler;
	private OneDimensionalMotionProfilerHDriveMain oneDProfilerH;
	
	public enum DriveMode {
		tankDrive, arcadeDrive, arcadeNegInertia
	}
	
	private Drive() {
		
		if (EnabledSubsystems.DRIVE_ENABLED == true || EnabledSubsystems.DUMB_DRIVE_ENABLED == true) {
			
			rightDrive = CTRECreator.createMasterTalon(RobotMap.RIGHT_TALON_ID);
			leftDrive = CTRECreator.createMasterTalon(RobotMap.LEFT_TALON_ID);
			rightDriveFollow = CTRECreator.createFollowerTalon(RobotMap.RIGHT_FOLLOW_TALON_ID, RobotMap.RIGHT_TALON_ID);
			leftDriveFollow = CTRECreator.createFollowerTalon(RobotMap.LEFT_FOLLOW_TALON_ID, RobotMap.LEFT_TALON_ID);
			hDrive = CTRECreator.createMasterTalon(RobotMap.H_TALON_ID);
			hFollowDrive = CTRECreator.createFollowerTalon(RobotMap.H_TALON_FOLLOW_ID, RobotMap.H_TALON_ID);
			
			leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, NO_TIMEOUT);
			leftDrive.config_kF(SLOT_0, 0, NO_TIMEOUT);
			leftDrive.config_kP(SLOT_0, P_LEFT, NO_TIMEOUT);
			leftDrive.config_kI(SLOT_0, I_LEFT, NO_TIMEOUT);
			leftDrive.config_kD(SLOT_0, D_LEFT, NO_TIMEOUT);
			
			leftDrive.enableVoltageCompensation(true);
			leftDrive.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL, NO_TIMEOUT);
			
			leftDrive.enableCurrentLimit(true);
			leftDrive.configPeakCurrentLimit(PEAK_CURRENT, NO_TIMEOUT);
			leftDrive.configPeakCurrentDuration(PEAK_CURRENT_DURATION, NO_TIMEOUT);
			leftDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, NO_TIMEOUT);
			
			leftDrive.setNeutralMode(NEUTRAL_MODE);
			leftDrive.setInverted(false);
			leftDriveFollow.setInverted(false);
			leftDrive.setSensorPhase(false);
			leftDriveFollow.setInverted(false);
			
			leftDrive.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, NO_TIMEOUT);
			leftDrive.configVelocityMeasurementWindow(32, NO_TIMEOUT);
			
			rightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, NO_TIMEOUT);
			rightDrive.config_kF(SLOT_0, 0, NO_TIMEOUT);
			rightDrive.config_kP(SLOT_0, P_RIGHT, NO_TIMEOUT);
			rightDrive.config_kI(SLOT_0, I_RIGHT, NO_TIMEOUT);
			rightDrive.config_kD(SLOT_0, D_RIGHT, NO_TIMEOUT);
			
			rightDrive.enableCurrentLimit(true);
			rightDrive.configPeakCurrentLimit(PEAK_CURRENT, NO_TIMEOUT);
			rightDrive.configPeakCurrentDuration(PEAK_CURRENT_DURATION, NO_TIMEOUT);
			rightDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, NO_TIMEOUT);
			
			rightDrive.setNeutralMode(NEUTRAL_MODE);
			rightDrive.setInverted(true);
			rightDriveFollow.setInverted(true);
			rightDrive.setSensorPhase(false);
			rightDriveFollow.setSensorPhase(false);
			
			rightDrive.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, NO_TIMEOUT);
			rightDrive.configVelocityMeasurementWindow(32, NO_TIMEOUT);
			
			hDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, NO_TIMEOUT);
			hDrive.config_kF(SLOT_0, 0, NO_TIMEOUT);
			hDrive.config_kP(SLOT_0, P_H, NO_TIMEOUT);
			hDrive.config_kI(SLOT_0, I_H, NO_TIMEOUT);
			hDrive.config_kD(SLOT_0, D_H, NO_TIMEOUT);
			
			hDrive.enableCurrentLimit(true);
			hDrive.configPeakCurrentLimit(PEAK_CURRENT, NO_TIMEOUT);
			hDrive.configPeakCurrentDuration(PEAK_CURRENT_DURATION, NO_TIMEOUT);
			hDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, NO_TIMEOUT);
			
			hDrive.setNeutralMode(NEUTRAL_MODE);			
			hDrive.setInverted(false);
			hDrive.setSensorPhase(false);
			
			hDrive.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, NO_TIMEOUT);
			hDrive.configVelocityMeasurementWindow(32, NO_TIMEOUT);

			rightEncoder = new TalonEncoder(rightDrive);
			leftEncoder = new TalonEncoder(leftDrive);
			hEncoder = new TalonEncoder(hDrive);
			
			leftDriveFollow.setNeutralMode(NEUTRAL_MODE);
			rightDriveFollow.setNeutralMode(NEUTRAL_MODE);
			hFollowDrive.setNeutralMode(NEUTRAL_MODE);
			
			CheesyDriveCalculationConstants.createDriveTypeCalculations();
			smartDashboardInit();
			
		}
		
		if (EnabledSubsystems.DUMB_DRIVE_ENABLED) {
			leftDrive.set(ControlMode.PercentOutput, 0);
			rightDrive.set(ControlMode.PercentOutput, 0);
			hDrive.set(ControlMode.PercentOutput, 0);
		} else {
			leftDrive.set(ControlMode.Velocity, 0);
			rightDrive.set(ControlMode.Velocity, 0);
			hDrive.set(ControlMode.Velocity, 0);
		}
	}
	
	public Speed currentMaxSpeed() {
		return MAX_SPEED;
	}
	
	public Speed currentMaxHSpeed() {
		return MAX_SPEED_H;
	}
	
	public void arcadeDrive(double move, double turn, double strafeRaw) {
		
		double[] motorPercents = new double[2];
		motorPercents = DriveTypeCalculations.arcadeDrive(move, turn);
		
		double strafe = NRMath.limit(strafeRaw);
		tankDrive(motorPercents[0], motorPercents[1], strafe);
	}

	public void tankDrive(double left, double right, double strafe) {
		setMotorSpeedInPercent(left, right, strafe);	
	}
	
	public void arcadeDriveNegInertia(double move, double turn, double strafe) {
		double[] cheesyMotorPercents = new double[2];
		cheesyMotorPercents = DriveTypeCalculations.cheesyDrive(move, turn, oldTurn, false);
		
		oldTurn = turn;
		
		tankDrive(cheesyMotorPercents[0], cheesyMotorPercents[1], strafe);		
	}
	
	public Distance getLeftDistance() {
		if(leftDrive != null) {
		return new Distance(leftDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK);
		}		
		else {
			return Distance.ZERO;
		}
	}
	
	public Distance getRightDistance() {
		if(rightDrive != null) {
			return new Distance(rightDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK);
			}
		else {
			return Distance.ZERO;
		}
	}
	
	public Distance getHDistance() {
		if(hDrive != null) {
			return new Distance(hDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK_H);
		}
		else {
			return Distance.ZERO;
		}
	}
	
	public static Drive getInstance() {
		if(singleton == null)
			init();
		return singleton;
	}
	
	public synchronized static void init() {
		if(singleton == null) {
			singleton = new Drive();
			singleton.setJoystickCommand(new DriveJoystickCommand());
		}
	}
	
	public void setMotorSpeedInPercent(double left, double right, double strafe) {
		//hDrive.set(ControlMode.PercentOutput, strafe);
		setMotorSpeed(currentMaxSpeed().mul(left), currentMaxSpeed().mul(right), currentMaxHSpeed().mul(strafe));
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
				hDrive.set(hDrive.getControlMode(), hMotorSetpoint.div(currentMaxHSpeed()));
			} else {
				hDrive.set(hDrive.getControlMode(), hMotorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND));
			}	
		}
	}
	
	/**
	 * Gets the historical position of the left talon
	 * 
	 * @param deltaTime
	 *            How long ago to look
	 * @return current position of talon
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
	 * @return current position of the talon
	 */
	public Distance getHistoricalRightPosition(Time deltaTime) {
		if (rightEncoder != null)
			return rightEncoder.getPosition(deltaTime);
		return Distance.ZERO;
	}
	
	public Distance getHistoricalHPosition(Time deltaTime) {
		if (hEncoder != null)
			return hEncoder.getPosition(deltaTime);
		return Distance.ZERO;
	}
	
	public double getRightCurrent() {
		if (rightDrive != null) {
			return Math.abs(rightDrive.getOutputCurrent());
		}
		return 0;
	}
	
	public double getLeftCurrent() {
		if (leftDrive != null) {
			return Math.abs(leftDrive.getOutputCurrent());
		}
		return 0;
	}
	
	public double getHCurrent() {
		if (hDrive != null) {
			return Math.abs(hDrive.getOutputCurrent());
		}
		return 0;
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
	public double pidGetLeft() {
		if (type == PIDSourceType.kRate) {
			return leftDrive.getSelectedSensorVelocity(PID_TYPE);
		} else {
			return leftDrive.getSelectedSensorPosition(PID_TYPE);
		}
	}

	@Override
	public double pidGetRight() {
		if (type == PIDSourceType.kRate) {
			return rightDrive.getSelectedSensorVelocity(PID_TYPE);
		} else {
			return rightDrive.getSelectedSensorPosition(PID_TYPE);
		}
	}
	
	@Override
	public double pidGetH() {
		if (type == PIDSourceType.kRate) {
			return hDrive.getSelectedSensorVelocity(PID_TYPE);
		} else {
			return hDrive.getSelectedSensorPosition(PID_TYPE);
		}
	}
	
	public Speed getLeftVelocity() {
		if (leftDrive != null)
			return new Speed(leftDrive.getSelectedSensorVelocity(SLOT_0), Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	public Speed getRightVelocity() {
		if(rightDrive != null)
			return new Speed(rightDrive.getSelectedSensorVelocity(SLOT_0), Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	public Speed getHVelocity() {
		if(hDrive != null)
			return new Speed(hDrive.getSelectedSensorVelocity(SLOT_0), Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	public void smartDashboardInit() {
		SmartDashboard.putNumber("Left P Value: ", P_LEFT);
		SmartDashboard.putNumber("Left I Value: ", I_LEFT);
		SmartDashboard.putNumber("Left D Value: ", D_LEFT);
		
		SmartDashboard.putNumber("Right P Value: ", P_RIGHT);
		SmartDashboard.putNumber("Right I Value: ", I_RIGHT);
		SmartDashboard.putNumber("Right D Value: ", D_RIGHT);
		
		SmartDashboard.putNumber("H P Value: ", P_H);
		SmartDashboard.putNumber("H I Value: ", I_H);
		SmartDashboard.putNumber("H D Value: ", D_H);
		
		SmartDashboard.putNumber("kVOneD Value: ", kVOneD);
		SmartDashboard.putNumber("kAOneD Value: ", kAOneD);
		SmartDashboard.putNumber("kPOneD Value: ", kPOneD);
		SmartDashboard.putNumber("kIOneD Value: ", kIOneD);
		SmartDashboard.putNumber("kDOneD Value: ", kDOneD);
		SmartDashboard.putNumber("HkpOneDAssist: ", HkpOneDAssist);
		SmartDashboard.putNumber("HkiOneDAssist: ", HkiOneDAssist);
		SmartDashboard.putNumber("HkdOneDAssist: ", HkdOneDAssist);
		SmartDashboard.putNumber("kP_thetaOneD Value: ", kP_thetaOneD);
		
		SmartDashboard.putNumber("Distance to Profile Feet", 0);
		SmartDashboard.putNumber("Drive Percent", 0);
		SmartDashboard.putNumber("Drive Percent H", 0);
		
		SmartDashboard.putNumber("Angle To Turn", 0);
	}
	
	@Override
	public void smartDashboardInfo() {
		SmartDashboard.putNumber("Drive Left Current", leftDrive.getOutputCurrent());
		SmartDashboard.putNumber("Drive Right Current", rightDrive.getOutputCurrent());
		SmartDashboard.putNumber("Drive H Current", hDrive.getOutputCurrent());
		
		SmartDashboard.putString("Drive Left Speed vs Set Speed: ", getLeftVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		SmartDashboard.putString("Drive Right Speed vs Set Speed: ", -getRightVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + -rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		SmartDashboard.putString("Drive H Speed vs Set Speed: ", getHVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + hMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		
		SmartDashboard.putNumber("Drive Left Voltage", leftDrive.getMotorOutputVoltage());
		SmartDashboard.putNumber("Drive Right Voltage", rightDrive.getMotorOutputVoltage());
		SmartDashboard.putNumber("Drive H Voltage", hDrive.getMotorOutputVoltage());
		
		if (Gyro.chosenGyro.equals(ChosenGyro.NavX)) {
			SmartDashboard.putNumber("Gyro Yaw", NavX.getInstance().getYaw().get(Angle.Unit.DEGREE));
		} else {
			SmartDashboard.putNumber("Gyro Yaw", Pigeon.getPigeon(getPigeonTalon()).getYaw().get(Angle.Unit.DEGREE));
		}
		
		P_LEFT = SmartDashboard.getNumber("Left P Value: ", P_LEFT);
		I_LEFT = SmartDashboard.getNumber("Left I Value: ", I_LEFT);
		D_LEFT = SmartDashboard.getNumber("Left D Value: ", D_LEFT);
		
		P_RIGHT = SmartDashboard.getNumber("Right P Value: ", P_RIGHT);
		I_RIGHT = SmartDashboard.getNumber("Right I Value: ", I_RIGHT);
		D_RIGHT = SmartDashboard.getNumber("Right D Value: ", D_RIGHT);
		
		P_H = SmartDashboard.getNumber("H P Value: ", P_H);
		I_H = SmartDashboard.getNumber("H I Value: ", I_H);
		D_H = SmartDashboard.getNumber("H D Value: ", D_H);
		
		leftDrive.config_kP(SLOT_0, SmartDashboard.getNumber("Left P Value: ", P_LEFT), NO_TIMEOUT);
		leftDrive.config_kI(SLOT_0, SmartDashboard.getNumber("Left I Value: ", I_LEFT), NO_TIMEOUT);
		leftDrive.config_kD(SLOT_0, SmartDashboard.getNumber("Left D Value: ", D_LEFT), NO_TIMEOUT);
		
		rightDrive.config_kP(SLOT_0, SmartDashboard.getNumber("Right P Value: ", P_RIGHT), NO_TIMEOUT);
		rightDrive.config_kI(SLOT_0, SmartDashboard.getNumber("Right I Value: ", I_RIGHT), NO_TIMEOUT);
		rightDrive.config_kD(SLOT_0, SmartDashboard.getNumber("Right D Value: ", D_RIGHT), NO_TIMEOUT);
		
		hDrive.config_kP(SLOT_0, SmartDashboard.getNumber("H P Value: ", P_H), NO_TIMEOUT);
		hDrive.config_kI(SLOT_0, SmartDashboard.getNumber("H I Value: ", I_H), NO_TIMEOUT);
		hDrive.config_kD(SLOT_0, SmartDashboard.getNumber("H D Value: ", D_H), NO_TIMEOUT);
		
		kVOneD = SmartDashboard.getNumber("kVOneD Value: ", kVOneD);
		kAOneD = SmartDashboard.getNumber("kAOneD Value: ", kAOneD);
		kPOneD = SmartDashboard.getNumber("kPOneD Value: ", kPOneD);
		kIOneD = SmartDashboard.getNumber("kIOneD Value: ", kIOneD);
		kDOneD = SmartDashboard.getNumber("kDOneD Value: ", kDOneD);
		HkpOneDAssist = SmartDashboard.getNumber("HkpOneDAssist: ", HkpOneDAssist);
		HkiOneDAssist = SmartDashboard.getNumber("HkiOneDAssist: ", HkiOneDAssist);
		HkdOneDAssist = SmartDashboard.getNumber("HkdOneDAssist: ", HkdOneDAssist);
		kP_thetaOneD = SmartDashboard.getNumber("kP_thetaOneD Value: ", kP_thetaOneD);
		
		oneDProfileHMain = new Distance(SmartDashboard.getNumber("Distance to Profile Feet", 0), Distance.Unit.FOOT);
		oneDProfileHAssist = new Distance(SmartDashboard.getNumber("Distance to Profile Feet", 0), Distance.Unit.FOOT);
		drivePercent = SmartDashboard.getNumber("Drive Percent", 0);
		drivePercentH = SmartDashboard.getNumber("Drive Percent H", 0);
		angleToTurn = new Angle(SmartDashboard.getNumber("Angle To Turn", 0), Angle.Unit.DEGREE);
	}

	public void periodic() {
	}

	public void disable() {
		Drive.getInstance().setMotorSpeed(Speed.ZERO, Speed.ZERO, Speed.ZERO);
		Command c = getCurrentCommand();
		if (c != null) {
			c.cancel();
		}
	}
	
	public void enableOneDProfiler(Distance dist) {
		oneDProfiler = new OneDimensionalMotionProfilerTwoMotorHDrive(this, this, kVOneD, kAOneD, kPOneD, kIOneD, kDOneD, kP_thetaOneD, HkpOneDAssist, HkiOneDAssist, HkdOneDAssist);
		oneDProfiler.setTrajectory(new OneDimensionalTrajectoryRamped(dist.get(Distance.Unit.MAGNETIC_ENCODER_TICK), 
				MAX_SPEED.mul(drivePercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND), 
				MAX_ACC.mul(ACCEL_PERCENT).get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND)));
		oneDProfiler.enable();
	}
	
	public void enableOneDProfilerH (Distance dist) {
		oneDProfilerH = new OneDimensionalMotionProfilerHDriveMain(this, this, H_kVOneD, H_kAOneD, H_kPOneD, H_kIOneD, H_kDOneD, H_kP_thetaOneD);
		oneDProfilerH.setTrajectory(new OneDimensionalTrajectoryRamped(dist.get(Distance.Unit.MAGNETIC_ENCODER_TICK_H),
				MAX_SPEED_H.mul(drivePercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_H,Time.Unit.HUNDRED_MILLISECOND), 
				MAX_ACC_H.mul(ACCEL_PERCENT).get(Distance.Unit.MAGNETIC_ENCODER_TICK_H, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND)));
		oneDProfilerH.enable();
	}
	
	public void disableOneDProfiler() {
		oneDProfiler.disable();
	}
	
	public void disableOneDProfilerH() {
		oneDProfilerH.disable();
	}

	@Override
	public void pidWrite(double outputLeft, double outputRight, double outputH) {
		setMotorSpeedInPercent(outputLeft, outputRight, outputH);
		
	}

	public TalonSRX getPigeonTalon() {
		return hFollowDrive;
	}

}
