package edu.nr.robotics.subsystems.drive;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.driving.DriveTypeCalculations;
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
import edu.nr.robotics.subsystems.EnabledSybsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {
	
	private static Drive singleton;
	
	public static final double WHEEL_DIAMETER_INCHES = 3.5;
	public static final Distance WHEEL_BASE = new Distance(27, Distance.Unit.INCH); //TODO: find for real
	
	public static final Speed MAX_SPEED_LEFT = new Speed(12.087, Distance.Unit.FOOT, Time.Unit.SECOND);
	public static final Speed MAX_SPEED_RIGHT = new Speed(12.079, Distance.Unit.FOOT, Time.Unit.SECOND);
	public static final Acceleration MAX_ACC = new Acceleration(31.53, Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND, Time.Unit.SECOND);
	public static final Jerk MAX_JERK = new Jerk(0, Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND, Time.Unit.SECOND);
	
	public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0.1081; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0.1067; //This is 0 to 1 number
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0.07377;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0.07394;
	
	
	public static final double MAX_DRIVE_CURRENT = 25; //in amps, maximum current to prevent the main breaker from cutting power
	public static final double ABOVE_MAX_CURRENT_DRIVE_PERCENT = 0.4; //if the max current is reached, it will run at this percent voltage instead
	
	public static boolean isQuickTurn = false; 
	
	private CANTalon leftDrive, rightDrive, rightDriveFollow, leftDriveFollow;
	private TalonEncoder leftEncoder, rightEncoder;
	
	//The speed in RPM that the motors are supposed to be running at... they get set later
	private Speed leftMotorSetpoint = Speed.ZERO;
	private Speed rightMotorSetpoint = Speed.ZERO;
	
	//TODO: get FPID values 
	public static final double P_RIGHT = 0;
	public static final double I_RIGHT = 0;
	public static final double D_RIGHT = 0;
	
	public static final double P_LEFT = 0;
	public static final double I_LEFT = 0;
	public static final double D_LEFT = 0;
	
	public static final int TICKS_PER_REV_2017 = 2048; // For 2017 Robot
	public static final int TICKS_PER_REV_TEST = 256; // For Test Bot
	
	PIDSourceType type = PIDSourceType.kRate;
	
	public static final double kP = 0;
	public static final double kD = 0;
	public static final double kP_theta = 0;
	
	private OneDimensionalMotionProfilerTwoMotor oneDProfiler;
	private TwoDimensionalMotionProfilerPathfinder twoDProfiler;
	
	public enum DriveMode {
		tankDrive, arcadeDrive, cheesyDrive
	}

	private Drive() {
		
		if(EnabledSybsystems.DRIVE_ENABLED || EnabledSybsystems.DUMB_DRIVE_ENABLED) {
			
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
			rightDrive.reverseSensor(true);
			rightDrive.enable();

			rightEncoder = new TalonEncoder(rightDrive);
			leftEncoder = new TalonEncoder(leftDrive);
			
			leftDriveFollow.changeControlMode(TalonControlMode.Follower);
			leftDriveFollow.enableBrakeMode(true);
			
			rightDriveFollow.changeControlMode(TalonControlMode.Follower);
			rightDriveFollow.enableBrakeMode(true);
			
			if (EnabledSybsystems.DUMB_DRIVE_ENABLED) {
				leftDrive.changeControlMode(TalonControlMode.PercentVbus);
				rightDrive.changeControlMode(TalonControlMode.PercentVbus);
			} else {
				leftDrive.changeControlMode(TalonControlMode.Speed);
				rightDrive.changeControlMode(TalonControlMode.Speed);
			}
			
			CheesyDriveCalculationConstants.createDriveTypeCalculations();
		
			SmartDashboard.putNumber("Drive Percent", 0);
			SmartDashboard.putNumber("Distance to Profile in Feet", 0);
		}
		
	}
	
	/**
	 * 
	 * @return in RPS
	 */
	public double currentMaxSpeedAve() {
		return (MAX_SPEED_LEFT.add(MAX_SPEED_RIGHT)).mul(0.5).get(Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND);
	}
	
	public Speed currentMaxSpeedLeft() {
		return MAX_SPEED_LEFT;
	}
	
	public Speed currentMaxSpeedRight() {
		return MAX_SPEED_RIGHT;
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
		setMotorSpeed(currentMaxSpeedLeft().mul(left), currentMaxSpeedRight().mul(right));
	}
	
	public void setMotorSpeed(Speed left, Speed right) {
		if (leftDrive != null && rightDrive != null) {
			
			if (getLeftCurrent() > MAX_DRIVE_CURRENT || getRightCurrent() > MAX_DRIVE_CURRENT) {
				leftMotorSetpoint = new Speed(currentMaxSpeedLeft().get(Distance.Unit.FOOT, Time.Unit.SECOND) * ABOVE_MAX_CURRENT_DRIVE_PERCENT, Distance.Unit.FOOT, Time.Unit.SECOND);
				rightMotorSetpoint = new Speed(currentMaxSpeedRight().get(Distance.Unit.FOOT, Time.Unit.SECOND) * -ABOVE_MAX_CURRENT_DRIVE_PERCENT, Distance.Unit.FOOT, Time.Unit.SECOND);
			}
			else {
				leftMotorSetpoint = left;
				rightMotorSetpoint = right.negate();
			}
			
			leftDrive.setF(((VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT * leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_LEFT) * 1023.0) / (new AngularSpeed(leftMotorSetpoint.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.HUNDRED_MILLISECOND), Angle.Unit.ROTATION, Time.Unit.HUNDRED_MILLISECOND).get(Angle.Unit.MAGNETIC_ENCODER_NATIVE_UNITS, Time.Unit.HUNDRED_MILLISECOND) / 4));
			rightDrive.setF(((VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT * rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND) - MIN_MOVE_VOLTAGE_PERCENT_RIGHT) * 1023.0) / (new AngularSpeed(rightMotorSetpoint.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.HUNDRED_MILLISECOND), Angle.Unit.ROTATION, Time.Unit.HUNDRED_MILLISECOND).get(Angle.Unit.MAGNETIC_ENCODER_NATIVE_UNITS, Time.Unit.HUNDRED_MILLISECOND) / 4));
			
			
			if (leftDrive.getControlMode() == TalonControlMode.PercentVbus) {
				leftDrive.set(leftMotorSetpoint.div(currentMaxSpeedLeft()));
			} else {
				leftDrive.set(leftMotorSetpoint.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE));
			}
			if (rightDrive.getControlMode() == TalonControlMode.PercentVbus) {
				rightDrive.set(rightMotorSetpoint.div(currentMaxSpeedRight()));
			} else {
				rightDrive.set(rightMotorSetpoint.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE));
			}
		}
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
			return -leftDrive.getSpeed() / 60;
		} else {
			return leftDrive.getPosition();
		}
	}

	@Override
	public double pidGetRight() {
		if (type == PIDSourceType.kRate) {
			return -rightDrive.getSpeed() / 60;
		} else {
			return rightDrive.getPosition();
		}
	}
	
	public Speed getLeftSpeed() {
		if (leftDrive != null)
			return new Speed(leftDrive.getSpeed(), Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE);
		return Speed.ZERO;
	}
	
	public Speed getRightSpeed() {
		if(rightDrive != null)
			return new Speed(leftDrive.getSpeed(), Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE);
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
		SmartDashboard.putString("Drive Right Speed vs Set Speed: ", getRightSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + -rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		
		SmartDashboard.putNumber("Drive Left Voltage", leftDrive.getOutputVoltage());
		SmartDashboard.putNumber("Drive Right Voltage", rightDrive.getOutputVoltage());
		
	//martDashboard.putNumber("Left F Value: ", F_LEFT);
		SmartDashboard.putNumber("Left P Value: ", P_LEFT);
		SmartDashboard.putNumber("Left I Value: ", I_LEFT);
		SmartDashboard.putNumber("Left D Value: ", D_LEFT);
		
		//SmartDashboard.putNumber("Right F Value: ", F_RIGHT);
		SmartDashboard.putNumber("Right P Value: ", P_RIGHT);
		SmartDashboard.putNumber("Right I Value: ", I_RIGHT);
		SmartDashboard.putNumber("Right D Value: ", D_RIGHT);
	
		SmartDashboard.putNumber("kP Value: ", kP);
		SmartDashboard.putNumber("kD Value: ", kD);
		SmartDashboard.putNumber("kP_theta Value: ", kP_theta);
	
		SmartDashboard.putNumber("Left F", leftDrive.getF());
		SmartDashboard.putNumber("Right F", rightDrive.getF());
	}

	public void periodic() {
		
		leftDrive.setP(SmartDashboard.getNumber("Left P Value: ", P_LEFT));
		leftDrive.setI(SmartDashboard.getNumber("Left I Value: ", I_LEFT));
		leftDrive.setD(SmartDashboard.getNumber("Left D Value: ", D_LEFT));
		
		rightDrive.setP(SmartDashboard.getNumber("Right P Value: ", P_RIGHT));
		rightDrive.setI(SmartDashboard.getNumber("Right I Value: ", I_RIGHT));
		rightDrive.setD(SmartDashboard.getNumber("Right D Value: ", D_LEFT));
		
	}

	public void disable() {
		Drive.getInstance().setMotorSpeed(Speed.ZERO, Speed.ZERO);
		Command c = getCurrentCommand();
		if (c != null) {
			c.cancel();
		}
	}
	
	public void enableOneDProfiler(Distance distance) {
		oneDProfiler = new OneDimensionalMotionProfilerTwoMotor(this, this, 1 / currentMaxSpeedAve(), 
				0,//1 / MAX_ACC.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND, Time.Unit.SECOND), 
				kP, kD, kP_theta);
		oneDProfiler.setTrajectory(new OneDimensionalTrajectorySimple(distance.get(Distance.Unit.DRIVE_ROTATION), 
				currentMaxSpeedAve(),
				currentMaxSpeedAve() * SmartDashboard.getNumber("Drive Percent", 0), 
				MAX_ACC.mul(SmartDashboard.getNumber("Drive Percent", 0)).get(Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND, Time.Unit.SECOND)));
		oneDProfiler.enable();
	}

	public void enableTwoDProfiler() {
		
	}

	public void disableOneDProfiler() {
		oneDProfiler.disable();
	}
	
	public void disableTwoDProfiler() {
		//twoDProfiler.disable();
	}
}
