package edu.nr.robotics.subsystems.drive;

import java.time.zone.ZoneOffsetTransitionRule.TimeDefinition;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.NRMath;
import edu.nr.lib.driving.DriveTypeCalculations;
import edu.nr.lib.interfaces.TriplePIDOutput;
import edu.nr.lib.interfaces.TriplePIDSource;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import edu.nr.lib.motionprofiling.RampedDiagonalHTrajectory;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.lib.units.Distance.Unit;
import edu.nr.robotics.OI;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Drive extends Subsystem implements TriplePIDOutput, TriplePIDSource {

	private static Drive singleton;
	
	private TalonSRX leftDrive, rightDrive, leftDriveFollow, rightDriveFollow, pigeonTalon;
	
	public static final double ENC_TO_WHEEL_GEARING = 1.0;
	
	public static final double EFFECTIVE_WHEEL_DIAMETER_INCHES = 0.0 * 0.95833333333333333333333333333333333333333333333333333333333333333333;
	
	public static final Distance REAL_WHEEL_DIAMETER = new Distance(EFFECTIVE_WHEEL_DIAMETER_INCHES, Distance.Unit.INCH);
	
	public static final Distance EFFECTIVE_WHEEL_DIAMETER = new Distance(EFFECTIVE_WHEEL_DIAMETER_INCHES / ENC_TO_WHEEL_GEARING, Distance.Unit.INCH);
	
	public static final Speed MAX_SPEED_DRIVE = new Speed(0.0, Distance.Unit.FOOT, Time.Unit.SECOND);
	
	public static final Acceleration MAX_ACCEL_DRIVE = new Acceleration(0.0, Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND);
	
	public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0.0; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0.0; //This is 0 to 1 number
	
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0.0;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0.0;
	
	public static Time DRIVE_RAMP_RATE = new Time(0.0, Time.Unit.SECOND);
	
	public static double P_LEFT = 0.0;
	public static double I_LEFT = 0.0;
	public static double D_LEFT = 0.0;
		
	public static double P_RIGHT = 0.0;
	public static double I_RIGHT = 0.0;
	public static double D_RIGHT = 0.0;
	
	public static double kVOneD = 1 / MAX_SPEED_DRIVE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
	public static double kAOneD = 0.0;
	public static double kPOneD = 0.0;
	public static double kIOneD = 0.0;
	public static double kDOneD = 0.0;
	public static double kP_thetaOneD = 0.0;
	
	public static final double PROFILE_DRIVE_PERCENT = 0.0;
	
	public static final double ACCEL_PERCENT = 0.0;
	
	public static final double MAX_PROFILE_TURN_PERCENT = 0.0;
	public static final double MIN_PROFILE_TURN_PERCENT = 0.0; 
	
	public static final Distance END_THRESHOLD = new Distance(0, Distance.Unit.INCH);
	
	public static final Speed PROFILE_END_SPEED_THRESHOLD = new Speed(0, Distance.Unit.INCH, Time.Unit.SECOND);
	
	public static final Time PROFILE_TIME_THRESHOLD = new Time(0, Time.Unit.SECOND);
	
	public static final Angle DRIVE_ANGLE_THRESHOLD = new Angle(0, Angle.Unit.DEGREE);
	
	public static final Angle DRIVE_STOP_ANGLE = new Angle(0, Angle.Unit.DEGREE);
	
	private static final int PEAK_DRIVE_CURRENT = 0;
	private static final int PEAK_DRIVE_CURRENT_DURATION = 0;
	private static final int CONTINUOUS_CURRENT_LIMIT = 0;
	
	public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_DRIVE = VelocityMeasPeriod.Period_10Ms;
	public static final int VELOCITY_MEASUREMENT_WINDOW_DRIVE = 0;
	
	private static final int VOLTAGE_COMPENSATION_LEVEL = 0;
	
	public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
	
	public static final int PID_TYPE = 0;
	
	public static final int VEL_SLOT = 0;

	public static final int SLOT_0 = 0;
	public static final int SLOT_1 = 0;
	
	public static final int DEFAULT_TIMEOUT = 0;
	
	private Speed leftMotorSetpoint = Speed.ZERO;
	private Speed rightMotorSetpoint = Speed.ZERO;
	private double oldTurn;
	
	private PIDSourceType type = PIDSourceType.kRate;
	
	public static Distance xProfile;
	public static Distance yProfile;
	public static double drivePercent;
	public static Angle angleToTurn; 
	public static boolean exact;
	
	public static enum DriveMode {
		arcadeDrive, tankDrive, cheesyDrive
	}
	
	private Drive() {
		
		if(EnabledSubsystems.DRIVE_ENABLED) {
			
			leftDrive = CTRECreator.createMasterTalon(RobotMap.LEFT_DRIVE_MASTER_TALON);
			rightDrive = CTRECreator.createMasterTalon(RobotMap.RIGHT_DRIVE_MASTER_TALON);
			
			leftDriveFollow = CTRECreator.createFollowerTalon(RobotMap.LEFT_DRIVE_FOLLOWER_TALON, RobotMap.LEFT_DRIVE_MASTER_TALON );
			rightDriveFollow = CTRECreator.createFollowerTalon(RobotMap.RIGHT_DRIVE_FOLLOWER_TALON, RobotMap.RIGHT_DRIVE_MASTER_TALON);
			

			leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			leftDrive.configkF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
			leftDrive.config_kP(VEL_SLOT, P_LEFT, DEFAULT_TIMEOUT);
			leftDrive.config_kI(VEL_SLOT, I_LEFT, DEFAULT_TIMEOUT);
			leftDrive.config_kD(VEL_SLOT, D_LEFT, DEFAULT_TIMEOUT);
			leftDrive.setNeutralMode(NEUTRAL_MODE);
			leftDrive.setInverted(false);
			leftDrive.setSensorPhase(false);
			leftDriveFollow.setSensorPhase(false);
			
			leftDrive.enableVoltageCompensation(true);
			leftDrive.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL, DEFAULT_TIMEOUT);
			
			leftDrive.enableCurrentLimit(true);
			leftDrive.configPeakCurrentLimit(PEAK_DRIVE_CURRENT, DEFAULT_TIMEOUT);
			leftDrive.configPeakCurrentDuration(PEAK_DRIVE_CURRENT_DURATION, DEFAULT_TIMEOUT);
			leftDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, DEFAULT_TIMEOUT);
			
			leftDrive.configVelocityMeasurementPeriod(VELOCITY_MEASUREMENT_PERIOD_DRIVE, DEFAULT_TIMEOUT);
			leftDrive.configVelocityMeasurementWindow(VELOCITY_MEASUREMENT_WINDOW_DRIVE, DEFAULT_TIMEOUT);
			
			leftDrive.configClosedloopRamp(DRIVE_RAMP_RATE.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			leftDrive.configOpenloopRamp(DRIVE_RAMP_RATE.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			
			
			leftDrive.selectProfileSlot(VEL_SLOT, DEFAULT_TIMEOUT);
			leftDriveFollow.setNeutralMode(NEUTRAL_MODE);
			

			rightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			rightDrive.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
			rightDrive.config_kP(VEL_SLOT, P_RIGHT, DEFAULT_TIMEOUT);
			rightDrive.config_kI(VEL_SLOT, I_RIGHT, DEFAULT_TIMEOUT);
			rightDrive.config_kD(VEL_SLOT, D_RIGHT, DEFAULT_TIMEOUT);
			rightDrive.setNeutralMode(NEUTRAL_MODE);			
			rightDrive.setInverted(true);
			rightDriveFollow.setInverted(true);
			rightDrive.setSensorPhase(false);
			rightDriveFollow.setSensorPhase(false);
			
			rightDrive.enableVoltageCompensation(true);
			rightDrive.configVoltageCompSaturation(VOLTAGE_COMPENSATION_LEVEL, DEFAULT_TIMEOUT);
			
			rightDrive.enableCurrentLimit(true);
			rightDrive.configPeakCurrentLimit(PEAK_DRIVE_CURRENT, DEFAULT_TIMEOUT);
			rightDrive.configPeakCurrentDuration(PEAK_DRIVE_CURRENT_DURATION, DEFAULT_TIMEOUT);
			rightDrive.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, DEFAULT_TIMEOUT);
			
			rightDrive.configVelocityMeasurementPeriod(VELOCITY_MEASUREMENT_PERIOD_DRIVE, DEFAULT_TIMEOUT);
			rightDrive.configVelocityMeasurementWindow(VELOCITY_MEASUREMENT_WINDOW_DRIVE, DEFAULT_TIMEOUT);
			
			rightDrive.configClosedloopRamp(DRIVE_RAMP_RATE.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			rightDrive.configOpenloopRamp(DRIVE_RAMP_RATE.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
			
			rightDrive.selectProfileSlot(VEL_SLOT, DEFAULT_TIMEOUT);
						
			rightDriveFollow.setNeutralMode(NEUTRAL_MODE);
			
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
			if (EnabledSubsystems.DRIVE_ENABLED)
				singleton.setJoystickCommand(new DriveJoystickCommand());
		}
	}
	
	public Distance getLeftPosition() {
		if(leftDrive != null) {
			return new Distance(leftDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK_DRIVE);
		}		
		else {
			return Distance.ZERO;
		}
	}
	
	public Distance getRightPosition() {
		if(rightDrive != null) {
			return new Distance(rightDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK_DRIVE);
		}
		else {
			return Distance.ZERO;
		}
	}
	
	public Speed getLeftVelocity() {
		if (leftDrive != null)
			return new Speed(leftDrive.getSelectedSensorVelocity(VEL_SLOT), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	public Speed getRightVelocity() {
		if(rightDrive != null)
			return new Speed(rightDrive.getSelectedSensorVelocity(VEL_SLOT), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}
	
	public double getRightCurrent() {
		if (rightDrive != null) {
			return rightDrive.getOutputCurrent();
		}
		return 0;
	}
	
	public double getRightFollowCurrent() {
		if (rightDriveFollow != null) {
			return rightDriveFollow.getOutputCurrent();
		}
		return 0;
	}
	
	public double getLeftCurrent() {
		if (leftDrive != null) {
			return leftDrive.getOutputCurrent();
		}
		return 0;
	}
	
	public double getLeftFollowCurrent() {
		if (leftDriveFollow != null) {
			return leftDriveFollow.getOutputCurrent();
		}
		return 0;
	}
	
	public void setMotorSpeedInPercent(double left, double right) {
		leftDrive.set(ControlMode.PercentOutput, left);
		rightDrive.set(ControlMode.PercentOutput, right);

		setMotorSpeed(MAX_SPEED_DRIVE.mul(left), MAX_SPEED_DRIVE.mul(right));
	
	}
	
	public void setMotorSpeed(Speed left, Speed right) {
		if (leftDrive != null && rightDrive != null) {
			
			leftMotorSetpoint = left;
			rightMotorSetpoint = right;
			
			leftDrive.config_kF(VEL_SLOT, ((VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT * leftMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_LEFT) * 1023.0) / leftMotorSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND), DEFAULT_TIMEOUT);
			rightDrive.config_kF(VEL_SLOT, ((VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT * rightMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_RIGHT) * 1023.0) / rightMotorSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND), DEFAULT_TIMEOUT);
			
			
			if (EnabledSubsystems.DRIVE_DUMB_ENABLED) {
				leftDrive.set(leftDrive.getControlMode(), leftMotorSetpoint.div(MAX_SPEED_DRIVE));
				rightDrive.set(rightDrive.getControlMode(), rightMotorSetpoint.div(MAX_SPEED_DRIVE));
			} else {
				leftDrive.set(leftDrive.getControlMode(), leftMotorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND));
				rightDrive.set(rightDrive.getControlMode(), rightMotorSetpoint.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND));
			}
		}
	}

	
	public void setVoltageRamp(Time time) {
		leftDrive.configClosedloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
		rightDrive.configClosedloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
		leftDrive.configOpenloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
		rightDrive.configOpenloopRamp(time.get(Time.Unit.SECOND), DEFAULT_TIMEOUT);
	}
	
	public void tankDrive(double left, double right) { 
		setMotorSpeedInPercent(left, right);
	}
	
	public void arcadeDrive(double move, double turn) {
		
		double[] motorPercents = new double[2];
		motorPercents = DriveTypeCalculations.arcadeDrive(move, turn);
	}
	
	public TalonSRX getPigeonTalon() {
		return pigeonTalon;
	}
	
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
			return getInstance().getLeftVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
		} else {
			return getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE);
		}
	}
	
	@Override
	public double pidGetRight() {
		if (type == PIDSourceType.kRate) {
			return getInstance().getRightVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
		} else {
			return getInstance().getRightPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE);
		}
	}
	
	public void pidWrite(double outputLeft, double outputRight) {
		setMotorSpeedInPercent(outputLeft, outputRight);
	}
	
	
	
	
}