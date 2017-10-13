package edu.nr.robotics.subsystems.drive;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.sf2.units.Unit;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.driving.DriveTypeCalculations;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;
import edu.nr.lib.sensorhistory.TalonEncoder;
import edu.nr.lib.talons.TalonCreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Jerk;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSybsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {
	
private static Drive singleton;
	
	public static final Distance WHEEL_DIAMETER = new Distance(3.5, Distance.Unit.INCH); //not correct TODO: get actual value from thing
	public static final Distance WHEEL_BASE = new Distance(27, Distance.Unit.INCH); //TODO: find for real
	
	public static final Speed MAX_SPEED = new Speed(13.33, Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND); //TODO: Find for real
	public static final Acceleration MAX_ACC = new Acceleration(0, Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND);
	public static final Jerk MAX_JERK = new Jerk(0, Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND, Time.Unit.SECOND);
	
	public static final double MAX_DRIVE_CURRENT = 25; //in amps, maximum current to prevent the main breaker from cutting power
	public static final double ABOVE_MAX_CURRENT_DRIVE_PERCENT = 0.4; //if the max current is reached, it will run at this percent voltage instead
	
	public static boolean isQuickTurn = false; 
	
	private CANTalon leftDrive, rightDrive, rightDriveFollow, leftDriveFollow;
	private TalonEncoder leftEncoder, rightEncoder;
	
	//The speed in RPM that the motors are supposed to be running at... they get set later
	private Speed leftMotorSetpoint = Speed.ZERO;
	private Speed rightMotorSetpoint = Speed.ZERO;
	
	//TODO: get FPID values
	public static final double F_RIGHT = 1;
	public static final double P_RIGHT = 0;
	public static final double I_RIGHT = 0;
	public static final double D_RIGHT = 0;
	
	public static final double F_LEFT = 1;
	public static final double P_LEFT = 0;
	public static final double I_LEFT = 0;
	public static final double D_LEFT = 0;
	
	public static final int TICKS_PER_REV_2017 = 2048; // For 2017 Robot
	public static final int TICKS_PER_REV_TEST = 256; // For Test Bot
	
	public enum DriveMode {
		tankDrive, arcadeDrive, cheesyDrive
	}
	
	public Speed currentMaxSpeed() {
		return MAX_SPEED;
	}
	
	private Drive() {
		
		if(EnabledSybsystems.DRIVE_ENABLED) {
			
			/** 
			 * Make sure to create talons with TalonCreator now
			 */
			leftDrive = TalonCreator.createMasterTalon(RobotMap.DRIVE_LEFT);
			rightDrive = TalonCreator.createMasterTalon(RobotMap.DRIVE_RIGHT);
			
			leftDriveFollow = TalonCreator.createFollowerTalon(RobotMap.DRIVE_LEFT_FOLLOW, leftDrive.getDeviceID());
			rightDriveFollow = TalonCreator.createFollowerTalon(RobotMap.DRIVE_RIGHT_FOLLOW, rightDrive.getDeviceID());
			
			if (EnabledSybsystems.DUMB_DRIVE_ENABLED) {
				leftDrive.changeControlMode(TalonControlMode.PercentVbus);
				rightDrive.changeControlMode(TalonControlMode.PercentVbus);
			} else {
				leftDrive.changeControlMode(TalonControlMode.Speed);
				rightDrive.changeControlMode(TalonControlMode.Speed);
			}
			
			leftDrive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			leftDrive.setF(F_LEFT);
			leftDrive.setP(P_LEFT);
			leftDrive.setI(I_LEFT);
			leftDrive.setD(D_LEFT);
			leftDrive.enableBrakeMode(true);
			leftDrive.setEncPosition(0);
			leftDrive.reverseSensor(false);
			leftDrive.enable();
			leftDrive.configEncoderCodesPerRev(TICKS_PER_REV_TEST);
			
			rightDrive.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			rightDrive.setF(F_RIGHT);
			rightDrive.setP(P_RIGHT);
			rightDrive.setI(I_RIGHT);
			rightDrive.setD(D_RIGHT);
			rightDrive.enableBrakeMode(true);
			rightDrive.setEncPosition(0);
			rightDrive.reverseSensor(false);
			rightDrive.enable();
			rightDrive.configEncoderCodesPerRev(TICKS_PER_REV_TEST);

			rightEncoder = new TalonEncoder(rightDrive);
			leftEncoder = new TalonEncoder(leftDrive);
			
			leftDriveFollow.changeControlMode(TalonControlMode.Follower);
			leftDriveFollow.enableBrakeMode(true);
			
			rightDriveFollow.changeControlMode(TalonControlMode.Follower);
			rightDriveFollow.enableBrakeMode(true);
			
			CheesyDriveCalculationConstants.createDriveTypeCalculations();
		
		}
		
	}
	
	public void arcadeDrive(double move, double turn) {
		double[] motorPercents = new double[2];
		motorPercents = DriveTypeCalculations.arcadeDrive(move, turn);

		tankDrive(motorPercents[0], motorPercents[1]);
	}
	
	public void tankDrive(double left, double right) {
		setMotorSpeedInPercent(left, right);	
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
			
			if (getLeftCurrent() > MAX_DRIVE_CURRENT || getRightCurrent() > MAX_DRIVE_CURRENT) {
				leftMotorSetpoint = new Speed(currentMaxSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) * ABOVE_MAX_CURRENT_DRIVE_PERCENT, Distance.Unit.FOOT, Time.Unit.SECOND);
				rightMotorSetpoint = new Speed(currentMaxSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) * -ABOVE_MAX_CURRENT_DRIVE_PERCENT, Distance.Unit.FOOT, Time.Unit.SECOND);
			}
			else {
				leftMotorSetpoint = left;
				rightMotorSetpoint = right.negate();
			}
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
	
	public void setPIDSourceType(PIDSourceType pidSource) {
		
	}

	public PIDSourceType getPIDSourceType() {
		return null;
	}

	public double pidGetLeft() {
		return 0;
	}

	public double pidGetRight() {
		return 0;
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

	public void pidWrite(double outputLeft, double outputRight) {
		
	}

	public void smartDashboardInfo() {
		SmartDashboard.putNumber("Drive Left Current", leftDrive.getOutputCurrent());
		SmartDashboard.putNumber("Drive Right Current", rightDrive.getOutputCurrent());
		
		SmartDashboard.putString("Drive Left Speed vs Set Speed: ", getLeftSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		SmartDashboard.putString("Drive Right Speed vs Set Speed: ", getRightSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
		
	}

	public void periodic() {
		
	}

	public void disable() {
		
	}
	
	

}
