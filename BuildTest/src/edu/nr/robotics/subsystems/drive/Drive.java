package edu.nr.robotics.subsystems.drive;

import java.io.File;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.driving.DriveTypeCalculations;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;
import edu.nr.lib.motionprofiling.TwoDimensionalMotionProfilerPathfinder;
import edu.nr.lib.motionprofiling.TwoDimensionalMotionProfilerPathfinderModified;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Distance.Unit;
import edu.nr.lib.units.Jerk;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Waypoint;

public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {

	private static Drive singleton;

	public TalonSRX leftDrive, rightDrive, leftDriveFollow, rightDriveFollow, pigeonTalon;

	public static final double REAL_ENC_TICK_PER_INCH_DRIVE = 428;

	public static final double EFFECTIVE_ENC_TICK_PER_INCH_DRIVE = REAL_ENC_TICK_PER_INCH_DRIVE;

	public static final Distance WHEEL_DIAMETER = new Distance(6, Distance.Unit.INCH);
	public static final Distance WHEEL_DIAMETER_EFFECTIVE = new Distance(6, Distance.Unit.INCH);

	public static final Distance WHEEL_BASE = new Distance(24, Distance.Unit.INCH);//.mul(1.36);
	/**
	 * The maximum speed of the drive base
	 */
	public static final Speed MAX_SPEED_DRIVE = new Speed(12.864, Distance.Unit.FOOT, Time.Unit.SECOND);

	/**
	 * The maximum acceleration of the drive base
	 */
	public static final Acceleration MAX_ACCEL_DRIVE = new Acceleration(20, Distance.Unit.FOOT, Time.Unit.SECOND,
			Time.Unit.SECOND);

	/**
	 * 
	 */
	public static final Jerk MAX_JERK_DRIVE = new Jerk(100, Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND,
			Time.Unit.SECOND);

	/**
	 * Voltage percentage at which robot just starts moving
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0.0726;// 0.0571; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0.0691;// 0.0600; //This is 0 to 1 number

	/**
	 * The drive voltage-velocity curve slopes
	 */
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0.0788;// 0.0733; //TODO: Find drive voltage vs
																			// velocity curve
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0.0784;// 0.0726;

	/**
	 * The amount of time drive can go from 0 to 12 volts
	 */
	public static Time DRIVE_RAMP_RATE = new Time(0.05, Time.Unit.SECOND);

	/**
	 * The amount of time h drive can go from 0 to 12 volts
	 */
	public static Time H_DRIVE_RAMP_RATE = new Time(0.05, Time.Unit.SECOND);

	/**
	 * The CANTalon PID values for velocity
	 */
	public static double P_LEFT = 0.2;
	public static double I_LEFT = 0;
	public static double D_LEFT = 2.0;

	public static double P_RIGHT = 0.2;
	public static double I_RIGHT = 0;
	public static double D_RIGHT = 2.0;

	/**
	 * 1D Profiling kVAPID_theta loop constants
	 */
	public static double kVOneD = 1
			/ MAX_SPEED_DRIVE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
	public static double kAOneD = 0.0;// 0.0002;
	public static double kPOneD = 0.00002;
	public static double kIOneD = 0;
	public static double kDOneD = 0;
	public static double kP_thetaOneD = 0.02;

	public static double kVTwoD = 1
			/ MAX_SPEED_DRIVE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
	public static double kATwoD = 0.0002;
	public static double kPTwoD = 0.0000;
	public static double kITwoD = 0.0;
	public static double kDTwoD = 0.00000;
	public static double kP_thetaTwoD = 0.0;

	/**
	 * Percent driving during profiling
	 */
	public static final double PROFILE_DRIVE_PERCENT = 0.8;

	/**
	 * Percent accelerating during profiling
	 */
	public static final double ACCEL_PERCENT = 0.8;

	/**
	 * Multiplies joystick turn value
	 */
	public static double TURN_JOYSTICK_MULTIPLIER = 1.0;

	/**
	 * Multiplies joystick move value
	 */
	public static double MOVE_JOYSTICK_MULTIPLIER = 1.0;

	/**
	 * Max and min speed of turn during
	 */
	public static final double MAX_PROFILE_TURN_PERCENT = 1.0;
	public static final double MIN_PROFILE_TURN_PERCENT = 0.02;

	/**
	 * The position from the end at which profile position threshold takes effect
	 */
	public static final Distance END_THRESHOLD = new Distance(3, Distance.Unit.INCH);

	/**
	 * The speed the profile needs to be under to stop
	 */
	public static final Speed PROFILE_END_SPEED_THRESHOLD = MAX_SPEED_DRIVE.mul(MIN_PROFILE_TURN_PERCENT + 0.01);

	/**
	 * The angle within which the turning stops
	 */
	public static final Angle DRIVE_ANGLE_THRESHOLD = new Angle(2, Angle.Unit.DEGREE);

	/**
	 * The angle the robot turns to once disabled at full turn speed. Used for
	 * GyroCorrection ramped mode.
	 */
	public static final Angle DRIVE_STOP_ANGLE = new Angle(55, Angle.Unit.DEGREE); // TODO: Find angle that robot stops
																					// at when turning goes from 1 to 0

	/**
	 * Current ratings based on MAXI Circuit Breaker Model MX5
	 */
	private static final int PEAK_DRIVE_CURRENT = 80; // In amps
	private static final int PEAK_DRIVE_CURRENT_DURATION = 1000; // In milliseconds
	private static final int CONTINUOUS_CURRENT_LIMIT = 40; // In amps

	/**
	 * When Driving into an object, the current when the driving stops
	 */
	public static final double SWITCH_CURRENT_LIMIT = 70;

	/**
	 * the drive percent while driving into the switch
	 */
	public static final double SWITCH_DRIVE_PERCENT = 0.4; // TODO: determine switch drive percent

	public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_DRIVE = VelocityMeasPeriod.Period_10Ms; // TODO:
																												// Find
																												// measurement
																												// period
																												// of
																												// velocity
	public static final int VELOCITY_MEASUREMENT_WINDOW_DRIVE = 32; // TODO: Find this

	/**
	 * Voltage level considered 100% for calculation
	 */
	private static final int VOLTAGE_COMPENSATION_LEVEL = 12; // In volts

	/**
	 * Default neutral mode (brake or coast)
	 */
	public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;

	/**
	 * Type of PID. 0 = primary. 1 = cascade
	 */
	public static final int PID_TYPE = 0;

	/**
	 * Used to prevent magic numbers from showing up in code
	 */
	public static final int VEL_SLOT = 0;

	/**
	 * No timeout for talon configuration functions
	 */
	public static final int DEFAULT_TIMEOUT = 0;

	/**
	 * Tracking of the drive motor setpoints
	 */
	private Speed leftMotorSetpoint = Speed.ZERO;
	private Speed rightMotorSetpoint = Speed.ZERO;
	private double oldTurn;

	private PIDSourceType type = PIDSourceType.kRate;

	public static final Distance ROBOT_WIDTH = new Distance(27, Distance.Unit.INCH);
	public static final Distance ROBOT_WIDTH_FUNCTIONAL = new Distance(27, Distance.Unit.INCH);

	public static Distance xProfile = new Distance(11.67, Distance.Unit.FOOT);
	public static Distance yProfile = new Distance(5.38, Distance.Unit.FOOT);
	public static Angle endAngle = new Angle(0, Angle.Unit.DEGREE);
	public static String profileName = "ProfileName";
	public static double drivePercent = 0.4;
	public static double accelPercent = 0.6;
	public static Angle angleToTurn;

	private TwoDimensionalMotionProfilerPathfinder twoDProfiler;
	private Waypoint[] points;

	/**
	 * Possible drive mode selections
	 */
	public static enum DriveMode {
		arcadeDrive, tankDrive, cheesyDrive
	}

	private Drive() {
		if (EnabledSubsystems.DRIVE_ENABLED) {

			leftDrive = CTRECreator.createMasterTalon(RobotMap.LEFT_DRIVE);
			rightDrive = CTRECreator.createMasterTalon(RobotMap.RIGHT_DRIVE);

			leftDriveFollow = CTRECreator.createFollowerTalon(RobotMap.LEFT_DRIVE_FOLLOW, leftDrive.getDeviceID());
			rightDriveFollow = CTRECreator.createFollowerTalon(RobotMap.RIGHT_DRIVE_FOLLOW, rightDrive.getDeviceID());
			pigeonTalon = CTRECreator.createMasterTalon(RobotMap.PIGEON_TALON);

			if (EnabledSubsystems.DRIVE_DUMB_ENABLED) {
				leftDrive.set(ControlMode.PercentOutput, 0);
				rightDrive.set(ControlMode.PercentOutput, 0);
			} else {
				leftDrive.set(ControlMode.Velocity, 0);
				rightDrive.set(ControlMode.Velocity, 0);
			}

			leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			leftDrive.config_kF(VEL_SLOT, 0, DEFAULT_TIMEOUT);
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

			LimelightNetworkTable.getInstance();

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

	/**
	 * @return Current position of the left talon
	 */
	public Distance getLeftPosition() {
		if (leftDrive != null) {
			return new Distance(leftDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK_DRIVE);
		} else {
			return Distance.ZERO;
		}
	}

	/**
	 * @return Current position of the right talon
	 */
	public Distance getRightPosition() {
		if (rightDrive != null) {
			return new Distance(rightDrive.getSelectedSensorPosition(PID_TYPE), Unit.MAGNETIC_ENCODER_TICK_DRIVE);
		} else {
			return Distance.ZERO;
		}
	}

	/**
	 * @return Current velocity of the left talon
	 */
	public Speed getLeftVelocity() {
		if (leftDrive != null)
			return new Speed(leftDrive.getSelectedSensorVelocity(VEL_SLOT), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
					Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
	}

	/**
	 * @return Current velocity of the right talon
	 */
	public Speed getRightVelocity() {
		if (rightDrive != null)
			return new Speed(rightDrive.getSelectedSensorVelocity(VEL_SLOT), Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
					Time.Unit.HUNDRED_MILLISECOND);
		return Speed.ZERO;
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

	public double getRightFollowCurrent() {
		if (rightDriveFollow != null) {
			return rightDriveFollow.getOutputCurrent();
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

	public double getLeftFollowCurrent() {
		if (leftDriveFollow != null) {
			return leftDriveFollow.getOutputCurrent();
		}
		return 0;
	}

	public void setMotorSpeedInPercent(double left, double right) {
		/*
		 * leftDrive.set(ControlMode.PercentOutput, left);
		 * rightDrive.set(ControlMode.PercentOutput, right);
		 */

		setMotorSpeed(MAX_SPEED_DRIVE.mul(left), MAX_SPEED_DRIVE.mul(right));
	}

	public void setMotorSpeed(Speed left, Speed right) {
		if (leftDrive != null && rightDrive != null) {

			leftMotorSetpoint = left;
			rightMotorSetpoint = right;

			leftDrive.config_kF(VEL_SLOT,
					((VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT
							* leftMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
							+ MIN_MOVE_VOLTAGE_PERCENT_LEFT) * 1023.0)
							/ leftMotorSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
									Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);
			rightDrive.config_kF(VEL_SLOT,
					((VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT
							* rightMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND)
							+ MIN_MOVE_VOLTAGE_PERCENT_RIGHT) * 1023.0)
							/ rightMotorSetpoint.abs().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
									Time.Unit.HUNDRED_MILLISECOND),
					DEFAULT_TIMEOUT);

			if (EnabledSubsystems.DRIVE_DUMB_ENABLED) {
				leftDrive.set(leftDrive.getControlMode(), leftMotorSetpoint.div(MAX_SPEED_DRIVE));
				rightDrive.set(rightDrive.getControlMode(), rightMotorSetpoint.div(MAX_SPEED_DRIVE));
			} else {
				leftDrive.set(leftDrive.getControlMode(), leftMotorSetpoint
						.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND));
				rightDrive.set(rightDrive.getControlMode(), rightMotorSetpoint
						.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND));
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

	/**
	 * An arcade drive
	 * 
	 * @param move      The speed, from -1 to 1 (inclusive), that the robot should
	 *                  go at. 1 is max forward, 0 is stopped, -1 is max backward
	 * @param turn      The speed, from -1 to 1 (inclusive), that the robot should
	 *                  turn at. 1 is max right, 0 is stopped, -1 is max left
	 * @param strafeRaw The speed, from -1 to 1 (inclusive), that the robot should
	 *                  strafe at. 1 is max right, 0 is stopped, -1 is max left
	 */
	public void arcadeDrive(double move, double turn) {

		double[] motorPercents = new double[2];
		motorPercents = DriveTypeCalculations.arcadeDrive(move, turn);

		tankDrive(motorPercents[0], motorPercents[1]);
	}

	/**
	 * Uses 254's CheesyDrive to drive
	 * 
	 * @param move The speed, from -1 to 1 (inclusive), that the robot should go at.
	 *             1 is max forward, 0 is stopped, -1 is max backward
	 * @param turn The speed, from -1 to 1 (inclusive), that the robot should turn
	 *             at. 1 is max right, 0 is stopped, -1 is max left
	 */
	public void cheesyDrive(double move, double turn) {
		double[] cheesyMotorPercents = new double[2];
		cheesyMotorPercents = DriveTypeCalculations.cheesyDrive(move, turn, oldTurn, false);

		oldTurn = turn;

		tankDrive(cheesyMotorPercents[0], cheesyMotorPercents[1]);
	}

	public TalonSRX getPigeonTalon() {
		return pigeonTalon;
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
			return getInstance().getLeftVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
					Time.Unit.HUNDRED_MILLISECOND);
		} else {
			return getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE);
		}
	}

	@Override
	public double pidGetRight() {
		if (type == PIDSourceType.kRate) {
			return getInstance().getRightVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
					Time.Unit.HUNDRED_MILLISECOND);
		} else {
			return getInstance().getRightPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE);
		}
	}

	@Override
	public void pidWrite(double outputLeft, double outputRight) {
		setMotorSpeedInPercent(outputLeft, outputRight);
	}

	public void enableMotionProfiler(Distance distX, Distance distY, Angle endAngle, double maxVelPercent,
			double maxAccelPercent, String profileName) {
		File profileFile = new File("home/lvuser/" + profileName + ".csv");
		
		twoDProfiler = new TwoDimensionalMotionProfilerPathfinder(this, this, kVTwoD, kATwoD, kPTwoD, kITwoD, kDTwoD,
				kP_thetaTwoD,
				MAX_SPEED_DRIVE.mul(maxVelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
						Time.Unit.HUNDRED_MILLISECOND),
				MAX_ACCEL_DRIVE.mul(maxAccelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE,
						Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
				MAX_JERK_DRIVE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND,
						Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND),
				(int) (Math.PI * WHEEL_DIAMETER_EFFECTIVE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE)),
				WHEEL_DIAMETER.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE),
				WHEEL_BASE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE), false, profileFile);

		System.out.println(profileFile.getName());
		
		if (!profileFile.exists()) {
			System.out.println("distX: " + distX.get(Distance.Unit.FOOT) + "	distY: "
					+ distY.get(Distance.Unit.FOOT) + "	end Angle: " + endAngle.get(Angle.Unit.DEGREE));
			points = new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(1, 0, 0),
					new Waypoint(distX.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE),
							distY.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE), endAngle.get(Angle.Unit.RADIAN)) };
		}
			twoDProfiler.setTrajectory(points);
		
		twoDProfiler.enable();
		
	}

	public void disableProfiler() {
		twoDProfiler.disable();
	}

	public void smartDashboardInit() {
		SmartDashboard.putNumber("X Profile Feet: ", xProfile.get(Distance.Unit.FOOT));
		SmartDashboard.putNumber("Y Profile Feet: ", yProfile.get(Distance.Unit.FOOT));
		SmartDashboard.putNumber("Profile End Angle: ", endAngle.get(Angle.Unit.DEGREE));
		SmartDashboard.putString("Profile Name: ", profileName);
		SmartDashboard.putNumber("Drive Percent: ", PROFILE_DRIVE_PERCENT);
		SmartDashboard.putNumber("Drive Accel Percent: ", ACCEL_PERCENT);

		SmartDashboard.putNumber("ka: ", kATwoD);
		SmartDashboard.putNumber("kp: ", kPTwoD);
		SmartDashboard.putNumber("ki: ", kITwoD);
		SmartDashboard.putNumber("kd: ", kDTwoD);
		SmartDashboard.putNumber("kp theta: ", kP_thetaTwoD);

	}

	@Override
	public void smartDashboardInfo() {
		/*SmartDashboard.putString("Drive Left Current", getLeftCurrent() + " : " + getLeftFollowCurrent());
		SmartDashboard.putString("Drive Right Current", getRightCurrent() + " : " + getRightFollowCurrent());

		SmartDashboard.putString("Drive Left Velocity: ", getLeftVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND)
				+ " : " + leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		SmartDashboard.putString("Drive Right Velocity: ", getRightVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND)
				+ " : " + rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));

		SmartDashboard.putNumber("Drive Left Percent", leftMotorSetpoint.div(MAX_SPEED_DRIVE));
		SmartDashboard.putNumber("Drive Right Percent", rightMotorSetpoint.div(MAX_SPEED_DRIVE));

		SmartDashboard.putNumber("Drive Left Position", getLeftPosition().get(Distance.Unit.INCH));
		SmartDashboard.putNumber("Drive Right Position", getRightPosition().get(Distance.Unit.INCH));

		SmartDashboard.putNumber("Drive Left Encoder Position", leftDrive.getSelectedSensorPosition(PID_TYPE));
		SmartDashboard.putNumber("Drive Right Encoder Position", rightDrive.getSelectedSensorPosition(PID_TYPE));

		xProfile = new Distance(SmartDashboard.getNumber("X Profile Feet: ", 0), Distance.Unit.FOOT);
		yProfile = new Distance(SmartDashboard.getNumber("Y Profile Feet: ", 0), Distance.Unit.FOOT);
		endAngle = new Angle(SmartDashboard.getNumber("Profile End Angle: ", 0), Angle.Unit.DEGREE);
		profileName = SmartDashboard.getString("Profile Name: ", profileName);
		drivePercent = SmartDashboard.getNumber("Drive Percent: ", 0);
		accelPercent = SmartDashboard.getNumber("Drive Accel Percent: ", 0);
		System.out.println("Reaches SmartDashboardInfo");

		kATwoD = SmartDashboard.getNumber("ka: ", kATwoD);
		kPTwoD = SmartDashboard.getNumber("kp: ", kPTwoD);
		kITwoD = SmartDashboard.getNumber("ki: ", kITwoD);
		kDTwoD = SmartDashboard.getNumber("kd: ", kDTwoD);
		kP_thetaTwoD = SmartDashboard.getNumber("kp theta: ", kP_thetaTwoD);*/
	}
	
	public void periodic() {
		SmartDashboard.putString("Drive Left Current", getLeftCurrent() + " : " + getLeftFollowCurrent());
		SmartDashboard.putString("Drive Right Current", getRightCurrent() + " : " + getRightFollowCurrent());

		SmartDashboard.putString("Drive Left Velocity: ", getLeftVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND)
				+ " : " + leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		SmartDashboard.putString("Drive Right Velocity: ", getRightVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND)
				+ " : " + rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));

		SmartDashboard.putNumber("Drive Left Percent", leftMotorSetpoint.div(MAX_SPEED_DRIVE));
		SmartDashboard.putNumber("Drive Right Percent", rightMotorSetpoint.div(MAX_SPEED_DRIVE));

		SmartDashboard.putNumber("Drive Left Position", getLeftPosition().get(Distance.Unit.INCH));
		SmartDashboard.putNumber("Drive Right Position", getRightPosition().get(Distance.Unit.INCH));

		SmartDashboard.putNumber("Drive Left Encoder Position", leftDrive.getSelectedSensorPosition(PID_TYPE));
		SmartDashboard.putNumber("Drive Right Encoder Position", rightDrive.getSelectedSensorPosition(PID_TYPE));

		xProfile = new Distance(SmartDashboard.getNumber("X Profile Feet: ", 0), Distance.Unit.FOOT);
		yProfile = new Distance(SmartDashboard.getNumber("Y Profile Feet: ", 0), Distance.Unit.FOOT);
		endAngle = new Angle(SmartDashboard.getNumber("Profile End Angle: ", 0), Angle.Unit.DEGREE);
		profileName = SmartDashboard.getString("Profile Name: ", profileName);
		drivePercent = SmartDashboard.getNumber("Drive Percent: ", 0);
		accelPercent = SmartDashboard.getNumber("Drive Accel Percent: ", 0);

		kATwoD = SmartDashboard.getNumber("ka: ", kATwoD);
		kPTwoD = SmartDashboard.getNumber("kp: ", kPTwoD);
		kITwoD = SmartDashboard.getNumber("ki: ", kITwoD);
		kDTwoD = SmartDashboard.getNumber("kd: ", kDTwoD);
		kP_thetaTwoD = SmartDashboard.getNumber("kp theta: ", kP_thetaTwoD);
	}

	@Override
	public void disable() {
		setMotorSpeedInPercent(0, 0);
	}

	public void startDumbDrive() {
		if (leftDrive != null && rightDrive != null) {
			EnabledSubsystems.DRIVE_DUMB_ENABLED = true;
		}
	}

	public void endDumbDrive() {
		if (leftDrive != null && rightDrive != null) {
			EnabledSubsystems.DRIVE_DUMB_ENABLED = false;
		}
	}

}