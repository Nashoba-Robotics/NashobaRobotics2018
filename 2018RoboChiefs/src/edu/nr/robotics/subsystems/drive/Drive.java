package edu.nr.robotics.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.NRMath;
import edu.nr.lib.gyro.NavX;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.Gyro;
import edu.nr.lib.gyro.Pigeon;
import edu.nr.lib.gyro.Gyro.ChosenGyro;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;
import edu.nr.lib.interfaces.SmartDashboardSource;
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

public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {

	private static Drive singleton;
	
	private TalonSRX leftDrive, rightDrive, leftDriveFollow, rightDriveFollow, pigeonTalon;
	private TalonEncoder leftEncoder, rightEncoder;
		
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
	public static final Speed MAX_SPEED = new Speed(0, Distance.Unit.FOOT, Time.Unit.SECOND); //TODO: Find real max speed
	
	/**
	 * The maximum acceleration of the drive base
	 */
	public static final Acceleration MAX_ACCELERATION = new Acceleration(0, Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND, Time.Unit.SECOND); //TODO: Find real max acceleration
	
	/**
	 * The maximum jerk of the drive base used for 2D Motion Profiling
	 */
	public static final Jerk MAX_JERK = new Jerk(0, Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND, Time.Unit.SECOND, Time.Unit.SECOND); //TODO: Find real max jerk
	
	/**
	 * The distance between the left and right wheels of the drive train
	 */
	public static final Distance WHEEL_BASE = new Distance(0, Distance.Unit.INCH); //TODO: Find real drive wheel base

	/**
	 * Ticks per revolution of the drive encoders
	 */
	public static final int TICKS_PER_REV = 0; //TODO: Get real ticks per rev of the drive encoders
	
	/**
	 * The percent drive motors output after reaching max current
	 */
	public static final double ABOVE_MAX_CURRENT_DRIVE_PERCENT = 0; //TODO: Find real value of percent driving above max current
	
	/**
	 * The time driven at max current before switching to ABOVE_MAX_CURRENT_DRIVE_PERCENT
	 */
	public static final Time MAX_CURRENT_PERIOD = new Time(0, Time.Unit.SECOND);
	
	/**
	 * The drive voltage-velocity curve slopes
	 */
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0; //TODO: Find drive voltage vs velocity curve
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0;
	
	/**
	 * Voltage percentage at which robot just starts moving
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0; //This is 0 to 1 number
	
	/**
	 * The CANTalon PID values for velocity
	 */
	public static final double P_LEFT = 0; //TODO: Find left PID values
	public static final double I_LEFT = 0;
	public static final double D_LEFT = 0;
		
	public static final double P_RIGHT = 0; //TODO: Find right PID values
	public static final double I_RIGHT = 0;
	public static final double D_RIGHT = 0;
	
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
	public static final Time PROFILE_TIME_THRESHOLD = new Time(0, Time.Unit.MILLISECOND); //TODO: Find Drive profile time threshold
	
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
	 * Tracking of the drive motor setpoints
	 */
	private Speed leftMotorSetpoint = Speed.ZERO;
	private Speed rightMotorSetpoint = Speed.ZERO;

	/**
	 * Possible drive mode selections
	 */
	public static enum DriveMode {
		arcadeDrive, tankDrive
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
	 * 1D Profiling kVAPID_theta loop constants
	 */
	public static double kVOneD = 1 / MAX_SPEED.get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND);
	public static double kAOneD = 0;
	public static double kPOneD = 0;
	public static double kIOneD = 0;
	public static double kDOneD = 0;
	public static double kP_thetaOneD = 0;
	
	/**
	 * 2D Profiling kVAPID_theta loop constants
	 */
	public static double kVTwoD = 1 / MAX_SPEED.get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND);;
	public static double kATwoD = 0;
	public static double kPTwoD = 0;
	public static double kITwoD = 0;
	public static double kDTwoD = 0;
	public static double kP_thetaTwoD = 0;
	
	/**
	 * Motion profiling parameters (x is front-back, y is left-right)
	 */
	public static Distance xProfile;
	public static Distance yProfile;
	public static Angle endAngle;
	public static double drivePercent = 0;
		
	private Drive() {
		if (EnabledSubsystems.DRIVE_ENABLED) {

			leftDrive = CTRECreator.createMasterTalon(RobotMap.DRIVE_LEFT);
			rightDrive = CTRECreator.createMasterTalon(RobotMap.DRIVE_RIGHT);
			pigeonTalon = CTRECreator.createMasterTalon(0);//TODO: find real pigeon talon
			
			leftDriveFollow = CTRECreator.createFollowerTalon(RobotMap.TEMP_LEFT_TALON, leftDrive.getDeviceID());
			rightDriveFollow = CTRECreator.createFollowerTalon(RobotMap.TEMP_RIGHT_TALON, rightDrive.getDeviceID());
			
			if (EnabledSubsystems.DRIVE_DUMB_ENABLED) {
				leftDrive.set(ControlMode.PercentOutput, 0);
			} else {
				leftDrive.set(ControlMode.Velocity, 0);
			}
			
			leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, NO_TIMEOUT);
			leftDrive.config_kF(SLOT_0, 0, NO_TIMEOUT);
			leftDrive.config_kP(SLOT_0, P_LEFT, NO_TIMEOUT);
			leftDrive.config_kI(SLOT_0, I_LEFT, NO_TIMEOUT);
			leftDrive.config_kD(SLOT_0, D_LEFT, NO_TIMEOUT);
			leftDrive.setNeutralMode(NEUTRAL_MODE);
			leftDrive.setInverted(false);
			leftDrive.setSensorPhase(true);
						
			leftEncoder = new TalonEncoder(leftDrive);
			
			leftDriveFollow.setNeutralMode(NEUTRAL_MODE);
			
			if (EnabledSubsystems.DRIVE_DUMB_ENABLED) {
				rightDrive.set(ControlMode.PercentOutput, 0);
			} else {
				rightDrive.set(ControlMode.Velocity, 0);
			}
			
			rightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, NO_TIMEOUT);
			rightDrive.config_kF(SLOT_0, 0, NO_TIMEOUT);
			rightDrive.config_kP(SLOT_0, P_RIGHT, NO_TIMEOUT);
			rightDrive.config_kI(SLOT_0, I_RIGHT, NO_TIMEOUT);
			rightDrive.config_kD(SLOT_0, D_RIGHT, NO_TIMEOUT);
			rightDrive.setNeutralMode(NEUTRAL_MODE);			
			rightDrive.setInverted(false);
			rightDrive.setSensorPhase(false);
			
			rightEncoder = new TalonEncoder(rightDrive);
			
			rightDriveFollow.setNeutralMode(NEUTRAL_MODE);
			
			smartDashboardInit();
			
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
	
	public void arcadeDrive(double move, double turn) {
		move = NRMath.limit(move);
		turn = NRMath.limit(turn);
		double leftMotorSpeed, rightMotorSpeed;

		if (move > 0.0) {
			if (turn > 0.0) {
				leftMotorSpeed = move - turn;
				rightMotorSpeed = Math.max(move, turn);
			} else {
				leftMotorSpeed = Math.max(move, -turn);
				rightMotorSpeed = move + turn;
			}
		} else {
			if (turn > 0.0) {
				leftMotorSpeed = -Math.max(-move, turn);
				rightMotorSpeed = move + turn;
			} else {
				leftMotorSpeed = move - turn;
				rightMotorSpeed = -Math.max(-move, -turn);
			}
		}
	
		tankDrive(leftMotorSpeed, rightMotorSpeed);
	}
	
	public void tankDrive(double left, double right) {
		setMotorSpeedInPercent(left, right);
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
	 * Gets the historical position of the left talon
	 * 
	 * @param deltaTime
	 *            How long ago to look
	 * @return old position of talon
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
	 * @return old position of the talon
	 */
	public Distance getHistoricalRightPosition(Time deltaTime) {
		if (rightEncoder != null)
			return rightEncoder.getPosition(deltaTime);
		return Distance.ZERO;
	}
	
	public void setMotorSpeedInPercent(double left, double right) {
		setMotorSpeed(currentMaxSpeed().mul(left), currentMaxSpeed().mul(right));
	}
	
	public void setMotorSpeed(Speed left, Speed right) {
		if (leftDrive != null && rightDrive != null) {
			
			leftMotorSetpoint = left;
			rightMotorSetpoint = right.negate();
			
			leftDrive.config_kF(SLOT_0, ((VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT * leftMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_LEFT) * 1023.0) / (new AngularSpeed(leftMotorSetpoint.abs().get(Distance.Unit.DRIVE_ROTATION, Time.Unit.HUNDRED_MILLISECOND), Angle.Unit.ROTATION, Time.Unit.HUNDRED_MILLISECOND).get(Angle.Unit.MAGNETIC_ENCODER_TICKS, Time.Unit.HUNDRED_MILLISECOND)), NO_TIMEOUT);
			rightDrive.config_kF(SLOT_0, ((VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT * rightMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_RIGHT) * 1023.0) / (new AngularSpeed(rightMotorSetpoint.abs().get(Distance.Unit.DRIVE_ROTATION, Time.Unit.HUNDRED_MILLISECOND), Angle.Unit.ROTATION, Time.Unit.HUNDRED_MILLISECOND).get(Angle.Unit.MAGNETIC_ENCODER_TICKS, Time.Unit.HUNDRED_MILLISECOND)), NO_TIMEOUT);
			
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
	
	public Speed getHistoricalLeftVelocity(Time deltaTime) {
		if (leftEncoder != null)
			return new Speed(leftEncoder.getVelocity(deltaTime));
		return Speed.ZERO;
	}
	
	public Speed getHistoricalRightVelocity(Time deltaTime) {
		if (rightEncoder != null)
			return new Speed(rightEncoder.getVelocity(deltaTime));
		return Speed.ZERO;
	}
	
	public void startDumbDrive() {
		if (leftDrive != null && rightDrive != null) {
			if (rightDrive.getControlMode() != ControlMode.PercentOutput) {
				rightDrive.set(ControlMode.PercentOutput, getRightVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND));
			}
			if (leftDrive.getControlMode() != ControlMode.PercentOutput) {
				leftDrive.set(ControlMode.PercentOutput, getLeftVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK, Time.Unit.HUNDRED_MILLISECOND));
			}
		}
	}
	
	public void endDumbDrive() {
		if (leftDrive != null && rightDrive != null) {
			if (rightDrive.getControlMode() != ControlMode.Velocity) {
				rightDrive.set(ControlMode.Velocity, 0);
			}
			if (leftDrive.getControlMode() != ControlMode.Velocity) {
				leftDrive.set(ControlMode.Velocity, 0);
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
			//
		}
	}
	
	@Override
	public void smartDashboardInfo() {
		if (leftDrive != null && rightDrive != null) {
			if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_BASIC_ENABLED) {
				
				SmartDashboard.putString("Drive Current", getLeftCurrent() + " : " + getRightCurrent());
				
				if (Gyro.chosenGyro.equals(ChosenGyro.NavX)) {
					SmartDashboard.putNumber("Gyro Yaw", NavX.getInstance().getYaw().get(Angle.Unit.DEGREE));
				} else {
					SmartDashboard.putNumber("Gyro Yaw", Pigeon.getPigeon(getInstance().getPigeonTalon()).getYaw().get(Angle.Unit.DEGREE));
				}
				
				SmartDashboard.putNumber("Drive Left Percent", leftMotorSetpoint.div(currentMaxSpeed()));
				SmartDashboard.putNumber("Drive Right Percent", rightMotorSetpoint.div(currentMaxSpeed()));
				
				SmartDashboard.putString("Drive Left Velocity: ", getLeftVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
				SmartDashboard.putString("Drive Right Velocity: ", getRightVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
				
			}
			if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
				SmartDashboard.putData(this);
				SmartDashboard.putString("Drive Voltage",
						leftDrive.getMotorOutputVoltage() + " : " + rightDrive.getMotorOutputVoltage());
				SmartDashboard.putNumber("Drive Left Position", getLeftPosition().get(Distance.Unit.FOOT));
				SmartDashboard.putNumber("Drive Right Position", getRightPosition().get(Distance.Unit.FOOT));
				
				leftDrive.config_kP(SLOT_0, SmartDashboard.getNumber("Left P Value: ", P_LEFT), NO_TIMEOUT);
				leftDrive.config_kI(SLOT_0, SmartDashboard.getNumber("Left I Value: ", I_LEFT), NO_TIMEOUT);
				leftDrive.config_kD(SLOT_0, SmartDashboard.getNumber("Left D Value: ", D_LEFT), NO_TIMEOUT);
				
				rightDrive.config_kP(SLOT_0, SmartDashboard.getNumber("Right P Value: ", P_RIGHT), NO_TIMEOUT);
				rightDrive.config_kI(SLOT_0, SmartDashboard.getNumber("Right I Value: ", I_RIGHT), NO_TIMEOUT);
				rightDrive.config_kD(SLOT_0, SmartDashboard.getNumber("Right D Value: ", D_RIGHT), NO_TIMEOUT);
				
				kVOneD = SmartDashboard.getNumber("kVOneD Value: ", kVOneD);
				kAOneD = SmartDashboard.getNumber("kAOneD Value: ", kAOneD);
				kPOneD = SmartDashboard.getNumber("kPOneD Value: ", kPOneD);
				kIOneD = SmartDashboard.getNumber("kIOneD Value: ", kIOneD);
				kDOneD = SmartDashboard.getNumber("kDOneD Value: ", kDOneD);
				kP_thetaOneD = SmartDashboard.getNumber("kP_thetaOneD Value: ", kP_thetaOneD);
				
				kVTwoD = SmartDashboard.getNumber("kVTwoD Value: ", kVTwoD);
				kATwoD = SmartDashboard.getNumber("kATwoD Value: ", kATwoD);
				kPTwoD = SmartDashboard.getNumber("kPTwoD Value: ", kPTwoD);
				kITwoD = SmartDashboard.getNumber("kITwoD Value: ", kITwoD);
				kDTwoD = SmartDashboard.getNumber("kDTwoD Value: ", kDTwoD);
				kP_thetaTwoD = SmartDashboard.getNumber("kP_thetaTwoD Value: ", kP_thetaTwoD);
				
				xProfile = new Distance(SmartDashboard.getNumber("X Profile Feet", 0), Distance.Unit.FOOT);
				yProfile = new Distance(SmartDashboard.getNumber("Y Profile Feet", 0), Distance.Unit.FOOT);
				endAngle = new Angle(SmartDashboard.getNumber("End Degree", 0), Angle.Unit.DEGREE);
				drivePercent = SmartDashboard.getNumber("Drive Percent", 0);
			}
		}
	}
	
	@Override
	public void disable() {
		setMotorSpeedInPercent(0, 0);
	}
	
	public double getRightCurrent() {
		if (rightDrive != null) {
			return rightDrive.getOutputCurrent();
		}
		return 0;
	}
	
	public double getLeftCurrent() {
		if (leftDrive != null) {
			return leftDrive.getOutputCurrent();
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
	public void pidWrite(double outputLeft, double outputRight) {
		setMotorSpeed(currentMaxSpeed().mul(outputLeft),currentMaxSpeed().mul(outputRight));
	}
}
