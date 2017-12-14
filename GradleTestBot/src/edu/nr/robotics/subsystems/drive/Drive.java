package edu.nr.robotics.subsystems.drive;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.driving.DriveTypeCalculations;
import edu.nr.lib.gyro.Gyro;
import edu.nr.lib.gyro.NavX;
import edu.nr.lib.gyro.Pigeon;
import edu.nr.lib.gyro.Gyro.ChosenGyro;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import edu.nr.lib.motionprofiling.OneDimensionalTrajectorySimple;
import edu.nr.lib.motionprofiling.TwoDimensionalMotionProfilerPathfinder;
import edu.nr.lib.sensorhistory.TalonEncoder;
import edu.nr.lib.talons.TalonCreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.AngularSpeed;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Distance.Unit;
import edu.nr.lib.units.Jerk;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.OI;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Waypoint;

public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {
	
	private static Drive singleton;
	
	public static final double WHEEL_DIAMETER_INCHES = 3.5;
	public static final Distance WHEEL_BASE = new Distance(27, Distance.Unit.INCH); //TODO: find for real
	
	public static final Speed MAX_SPEED = new Speed(12.698, Distance.Unit.FOOT, Time.Unit.SECOND);
	public static final Acceleration MAX_ACC = new Acceleration(31.53, Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND, Time.Unit.SECOND);
	public static final Jerk MAX_JERK = new Jerk(813, Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND, Time.Unit.SECOND, Time.Unit.SECOND);
	
	public static final double ACCEL_PERCENT = 0.5;
	
	public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0.0924; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0.0567; //This is 0 to 1 number
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0.0717;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0.0739;
	
	public static double ampTimerStart = Timer.getFPGATimestamp();
	public static boolean ampTimerStarted = false;
	public static final double MAX_CURRENT_PERIOD = 0.5;
	
	public static final double MAX_DRIVE_CURRENT = 25; //in amps, maximum current while driving normally
	public static final double ABOVE_MAX_CURRENT_DRIVE_PERCENT = 0.8; //if the max current is reached, it will run at this percent voltage instead
	
	public static boolean isQuickTurn = false; 
	
	private CANTalon leftDrive, rightDrive, rightDriveFollow, leftDriveFollow;
	private TalonEncoder leftEncoder, rightEncoder;
	
	//The speed in RPM that the motors are supposed to be running at... they get set later
	public Speed leftMotorSetpoint = Speed.ZERO;
	public Speed rightMotorSetpoint = Speed.ZERO;
	
	//TODO: get FPID values 
	public static final double P_RIGHT = 1.5;
	public static final double I_RIGHT = 0;
	public static final double D_RIGHT = 0.15;
	
	public static final double P_LEFT = 1.5;
	public static final double I_LEFT = 0;
	public static final double D_LEFT = 0.15;
	
	public static final int TICKS_PER_REV_2017 = 2048; // For 2017 Robot
	public static final int TICKS_PER_REV_TEST = 256; // For Test Bot
	
	PIDSourceType type = PIDSourceType.kRate;
	
	//public static double kVOneD = 0.07226;
	public static double kVOneD = 1 / MAX_SPEED.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND);
	public static double kAOneD = 0.0018;
	public static double kPOneD = 0;//0.01
	public static double kIOneD = 0;
	public static double kDOneD = 0;
	public static double kP_thetaOneD = 0;
	
	public static double kVTwoD = 1 / MAX_SPEED.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND);;
	public static double kATwoD = 0;
	public static double kPTwoD = 0;
	public static double kITwoD = 0;
	public static double kDTwoD = 0;
	public static double kP_thetaTwoD = 0;
	
	public static Distance xProfile;
	public static Distance yProfile;
	public static Angle endAngle;
	public static double drivePercent = 0;
	
	private OneDimensionalMotionProfilerTwoMotor oneDProfiler;
	private TwoDimensionalMotionProfilerPathfinder twoDProfiler;
	
	public enum DriveMode {
		tankDrive, arcadeDrive, cheesyDrive
	}

	private Drive() {
		
		if(EnabledSubsystems.DRIVE_ENABLED || EnabledSubsystems.DUMB_DRIVE_ENABLED) {
			
			leftDrive = TalonCreator.createMasterTalon(RobotMap.DRIVE_LEFT);
			rightDrive = TalonCreator.createMasterTalon(RobotMap.DRIVE_RIGHT);
			
			leftDriveFollow = TalonCreator.createFollowerTalon(RobotMap.DRIVE_LEFT_FOLLOW, leftDrive.getDeviceID());
			rightDriveFollow = TalonCreator.createFollowerTalon(RobotMap.DRIVE_RIGHT_FOLLOW, rightDrive.getDeviceID());
			
			leftDrive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			leftDrive.setProfile(0);
			leftDrive.setF(0);
			leftDrive.setP(P_LEFT);
			leftDrive.setI(I_LEFT);
			leftDrive.setD(D_LEFT);
			leftDrive.configEncoderCodesPerRev(TICKS_PER_REV_TEST);
			leftDrive.enableBrakeMode(true);
			leftDrive.setEncPosition(0);
			leftDrive.reverseSensor(true);
			leftDrive.enable();
			
			rightDrive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			rightDrive.setProfile(0);
			rightDrive.setF(0);
			rightDrive.setP(P_RIGHT);
			rightDrive.setI(I_RIGHT);
			rightDrive.setD(D_RIGHT);
			rightDrive.configEncoderCodesPerRev(TICKS_PER_REV_TEST);
			rightDrive.enableBrakeMode(true);
			rightDrive.setEncPosition(0);
			rightDrive.reverseSensor(false);
			rightDrive.enable();

			rightEncoder = new TalonEncoder(rightDrive);
			leftEncoder = new TalonEncoder(leftDrive);
			
			leftDriveFollow.changeControlMode(TalonControlMode.Follower);
			leftDriveFollow.enableBrakeMode(true);
			
			rightDriveFollow.changeControlMode(TalonControlMode.Follower);
			rightDriveFollow.enableBrakeMode(true);
			
			if (EnabledSubsystems.DUMB_DRIVE_ENABLED) {
				leftDrive.changeControlMode(TalonControlMode.PercentVbus);
				rightDrive.changeControlMode(TalonControlMode.PercentVbus);
			} else {
				leftDrive.changeControlMode(TalonControlMode.Speed);
				rightDrive.changeControlMode(TalonControlMode.Speed);
			}
			
			CheesyDriveCalculationConstants.createDriveTypeCalculations();
		
			SmartDashboard.putNumber("Drive Percent", 0);
			SmartDashboard.putNumber("X Profile Feet", 0);
			SmartDashboard.putNumber("Y Profile Feet", 0);
			SmartDashboard.putNumber("End Degree", 0);
			
			SmartDashboard.putNumber("Left P Value: ", P_LEFT);
			SmartDashboard.putNumber("Left I Value: ", I_LEFT);
			SmartDashboard.putNumber("Left D Value: ", D_LEFT);
			
			SmartDashboard.putNumber("Right P Value: ", P_RIGHT);
			SmartDashboard.putNumber("Right I Value: ", I_RIGHT);
			SmartDashboard.putNumber("Right D Value: ", D_RIGHT);
		
			
			SmartDashboard.putNumber("kVOneD Value: ", kVOneD);
			SmartDashboard.putNumber("kAOneD Value: ", kAOneD);
			SmartDashboard.putNumber("kPOneD Value: ", kPOneD);
			SmartDashboard.putNumber("kIOneD Value: ", kIOneD);
			SmartDashboard.putNumber("kDOneD Value: ", kDOneD);
			SmartDashboard.putNumber("kP_thetaOneD Value: ", kP_thetaOneD);
			
			SmartDashboard.putNumber("kVTwoD Value: ", kVTwoD);
			SmartDashboard.putNumber("kATwoD Value: ", kATwoD);
			SmartDashboard.putNumber("kPTwoD Value: ", kPTwoD);
			SmartDashboard.putNumber("kITwoD Value: ", kITwoD);
			SmartDashboard.putNumber("kDTwoD Value: ", kDTwoD);
			SmartDashboard.putNumber("kP_thetaTwoD Value: ", kP_thetaTwoD);
		
			SmartDashboard.putNumber("Left F", leftDrive.getF());
			SmartDashboard.putNumber("Right F", rightDrive.getF());
			
		}
		
	}
	
	
	public void smartDashboardInit() {
		
	}
	
	public Speed currentMaxSpeed() {
		return MAX_SPEED;
	}
	
	public void arcadeDrive(double move, double turn) {
		double[] motorPercents = new double[2];
		motorPercents = DriveTypeCalculations.arcadeDrive(move, turn);
		
		tankDrive(motorPercents[0], motorPercents[1]);
	}
	
	public void tankDrive(double left, double right) {
		setMotorSpeedInPercent(left, right);	
	}
	
	public Distance getLeftDistance() {
		if(leftDrive != null) {
		return new Distance(leftDrive.getPosition(), Unit.DRIVE_ROTATION);
		}
		
		else
			return Distance.ZERO;
	}
	
	public Distance getRightDistance() {
		if(rightDrive != null) {
			return new Distance(rightDrive.getPosition(), Unit.DRIVE_ROTATION);
			}
		else {
			return Distance.ZERO;
		}
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
	public void cheesyDrive(double move, double turn) {
		double[] cheesyMotorPercents = new double[2];
		cheesyMotorPercents = DriveTypeCalculations.cheesyDrive(move, turn, isQuickTurn, false);
		
		tankDrive(cheesyMotorPercents[0], cheesyMotorPercents[1]);
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
	
	public void setMotorSpeedInPercent(double left, double right) {
		setMotorSpeed(currentMaxSpeed().mul(left), currentMaxSpeed().mul(right));
	}
	
	public void setMotorSpeed(Speed left, Speed right) {
		if (leftDrive != null && rightDrive != null) {
			
			if ((getLeftCurrent() > MAX_DRIVE_CURRENT || getRightCurrent() > MAX_DRIVE_CURRENT) && Math.abs(OI.getInstance().getArcadeMoveValue()) < ABOVE_MAX_CURRENT_DRIVE_PERCENT) { 
				if (!ampTimerStarted) {
					ampTimerStart = Timer.getFPGATimestamp();
					ampTimerStarted = true;
				}
			} else {
				ampTimerStarted = false;
			}
			
			if (ampTimerStarted && (Timer.getFPGATimestamp() - ampTimerStart) >= MAX_CURRENT_PERIOD) {
				leftMotorSetpoint = currentMaxSpeed().mul(ABOVE_MAX_CURRENT_DRIVE_PERCENT);
				rightMotorSetpoint = currentMaxSpeed().negate().mul(ABOVE_MAX_CURRENT_DRIVE_PERCENT);
			} else {
				leftMotorSetpoint = left;
				rightMotorSetpoint = right.negate();
			}
			
			leftDrive.setF(((VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT * leftMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_LEFT) * 1023.0) / (new AngularSpeed(leftMotorSetpoint.abs().get(Distance.Unit.DRIVE_ROTATION, Time.Unit.HUNDRED_MILLISECOND), Angle.Unit.ROTATION, Time.Unit.HUNDRED_MILLISECOND).get(Angle.Unit.MAGNETIC_ENCODER_NATIVE_UNITS, Time.Unit.HUNDRED_MILLISECOND) / 4));
			rightDrive.setF(((VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT * rightMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_RIGHT) * 1023.0) / (new AngularSpeed(rightMotorSetpoint.abs().get(Distance.Unit.DRIVE_ROTATION, Time.Unit.HUNDRED_MILLISECOND), Angle.Unit.ROTATION, Time.Unit.HUNDRED_MILLISECOND).get(Angle.Unit.MAGNETIC_ENCODER_NATIVE_UNITS, Time.Unit.HUNDRED_MILLISECOND) / 4));
			
			if (leftDrive.getControlMode() == TalonControlMode.PercentVbus) {
				leftDrive.set(leftMotorSetpoint.div(currentMaxSpeed()));
			} else {
				leftDrive.set(leftMotorSetpoint.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE));
			}
			if (rightDrive.getControlMode() == TalonControlMode.PercentVbus) {
				rightDrive.set(rightMotorSetpoint.div(currentMaxSpeed()));
			} else {
				rightDrive.set(rightMotorSetpoint.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE));
			}
		}
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
			return leftDrive.getSpeed() / 60;
		} else {
			return leftDrive.getPosition();
		}
	}

	@Override
	public double pidGetRight() {
		if (type == PIDSourceType.kRate) {
			return -rightDrive.getSpeed() / 60;
		} else {
			return -rightDrive.getPosition();
		}
	}
	
	public Speed getLeftSpeed() {
		if (leftDrive != null)
			return new Speed(leftDrive.getSpeed(), Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE);
		return Speed.ZERO;
	}
	
	public Speed getRightSpeed() {
		if(rightDrive != null)
			return new Speed(rightDrive.getSpeed(), Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE);
		return Speed.ZERO;
	}

	@Override
	public void pidWrite(double left, double right) {
		setMotorSpeedInPercent(left, right);
	}
	
	@Override
	public void smartDashboardInfo() {
		SmartDashboard.putNumber("Drive Left Current", leftDrive.getOutputCurrent());
		SmartDashboard.putNumber("Drive Right Current", rightDrive.getOutputCurrent());
		
		SmartDashboard.putString("Drive Left Speed vs Set Speed: ", getLeftSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		SmartDashboard.putString("Drive Right Speed vs Set Speed: ", -getRightSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + -rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		
		SmartDashboard.putNumber("Drive Left Voltage", leftDrive.getOutputVoltage());
		SmartDashboard.putNumber("Drive Right Voltage", rightDrive.getOutputVoltage());
		
		if (Gyro.chosenGyro.equals(ChosenGyro.NavX)) {
			SmartDashboard.putNumber("Gyro Yaw", NavX.getInstance().getYaw().get(Angle.Unit.DEGREE));
		} else {
			SmartDashboard.putNumber("Gyro Yaw", Pigeon.getInstance().getYaw().get(Angle.Unit.DEGREE));
		}
		
		leftDrive.setP(SmartDashboard.getNumber("Left P Value: ", P_LEFT));
		leftDrive.setI(SmartDashboard.getNumber("Left I Value: ", I_LEFT));
		leftDrive.setD(SmartDashboard.getNumber("Left D Value: ", D_LEFT));
		
		rightDrive.setP(SmartDashboard.getNumber("Right P Value: ", P_RIGHT));
		rightDrive.setI(SmartDashboard.getNumber("Right I Value: ", I_RIGHT));
		rightDrive.setD(SmartDashboard.getNumber("Right D Value: ", D_LEFT));
		
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

	public void periodic() {
	}

	public void disable() {
		Drive.getInstance().setMotorSpeed(Speed.ZERO, Speed.ZERO);
		Command c = getCurrentCommand();
		if (c != null) {
			c.cancel();
		}
	}
	
	public void enableOneDProfiler(Distance dist) {
		oneDProfiler = new OneDimensionalMotionProfilerTwoMotor(this, this, kVOneD, kAOneD, kPOneD, kIOneD, kDOneD, kP_thetaOneD);
		oneDProfiler.setTrajectory(new OneDimensionalTrajectorySimple(dist.get(Distance.Unit.FOOT), 
				MAX_SPEED.get(Distance.Unit.FOOT, Time.Unit.SECOND) * drivePercent, 
				MAX_ACC.mul(ACCEL_PERCENT).get(Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND)));
		oneDProfiler.enable();
	}

	public void enableTwoDProfiler(Distance xDist, Distance yDist, Angle endAng) {
		twoDProfiler = new TwoDimensionalMotionProfilerPathfinder(this, this, kVTwoD, kATwoD, kPTwoD, kITwoD, kDTwoD, kP_thetaTwoD,
				MAX_SPEED.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND) * drivePercent, 
				MAX_ACC.mul(ACCEL_PERCENT).get(Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND, Time.Unit.SECOND),
				MAX_JERK.mul(ACCEL_PERCENT).get(Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND, Time.Unit.SECOND, Time.Unit.SECOND) * ACCEL_PERCENT, 
				TICKS_PER_REV_TEST, WHEEL_DIAMETER_INCHES / 12, WHEEL_BASE.get(Distance.Unit.DRIVE_ROTATION), 
				false);
		
		Waypoint[] points = new Waypoint[] {
			new Waypoint(0, 0, 0),
			//new Waypoint(0.25, 0, 0),
			new Waypoint(xDist.get(Distance.Unit.DRIVE_ROTATION), yDist.get(Distance.Unit.DRIVE_ROTATION), endAng.get(Angle.Unit.RADIAN))
		};
		
		twoDProfiler.setTrajectory(points);
		twoDProfiler.enable();
	}

	public void disableOneDProfiler() {
		oneDProfiler.disable();
	}
	
	public void disableTwoDProfiler() {
		twoDProfiler.disable();
	}
}
