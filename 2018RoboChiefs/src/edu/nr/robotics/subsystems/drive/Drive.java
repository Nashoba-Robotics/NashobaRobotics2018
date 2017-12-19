package edu.nr.robotics.subsystems.drive;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.nr.lib.NRMath;
import edu.nr.lib.gyro.NavX;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.Gyro;
import edu.nr.lib.gyro.Pigeon;
import edu.nr.lib.gyro.Gyro.ChosenGyro;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;
import edu.nr.lib.sensorhistory.TalonEncoder;
import edu.nr.lib.talons.TalonCreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.AngularSpeed;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Jerk;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.OI;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {

	private static Drive singleton;
	
	private CANTalon leftTalon, rightTalon, tempLeftTalon, tempRightTalon;
	private TalonEncoder leftEncoder, rightEncoder;
	
	public static final double ABOVE_MAX_CURRENT_DRIVE_PERCENT = 0; //TODO: Find real value
	
	public static final double MAX_DRIVE_CURRENT = 0; //TODO: Find real value
	
	public static final double WHEEL_DIAMETER_INCHES = 0; //TODO: Find real drive wheel diameter
	
	public static final double DISTANCE_PER_REV_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
	
	public static final Distance WHEEL_DIAMETER = new Distance(WHEEL_DIAMETER_INCHES, Distance.Unit.INCH);

	static final Distance DISTANCE_PER_REV = WHEEL_DIAMETER.mul(Math.PI);

	public static final Speed MAX_SPEED = new Speed(0, Distance.Unit.FOOT, Time.Unit.SECOND); //TODO: Find real max speed
	
	public static final Acceleration MAX_ACCELERATION = new Acceleration(0, Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND, Time.Unit.SECOND); //TODO: Find real max acceleration
	
	public static final Jerk MAX_JERK = new Jerk(0, Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND, Time.Unit.SECOND, Time.Unit.SECOND); //TODO: Find real max jerk
	
	public static final Distance WHEEL_BASE = new Distance(0, Distance.Unit.INCH); //TODO: Find real drive wheel base

	public static final int TICKS_PER_REV = 0; //TODO: Get real ticks per rev
	
	private static final int NATIVE_UNITS_PER_REV = 4 * TICKS_PER_REV;
	
	public static double ampTimerStart = Timer.getFPGATimestamp();
	public static boolean ampTimerStarted = false;
	public static final double MAX_CURRENT_PERIOD = 0.5;
	
	//TODO: Find slope of voltage vs velocity
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0;
	
	//TODO: Find offset of voltage vs velocity graph
	public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0; //This is 0 to 1 number
	
	private Speed leftMotorSetpoint = Speed.ZERO;
	private Speed rightMotorSetpoint = Speed.ZERO;
	
	//TODO: Find left FPID values
	public static final double F_LEFT = 0;//1023.0/(MAX_LOW_GEAR_SPEED.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.HUNDRED_MILLISECOND) * NATIVE_UNITS_PER_REV);
	public static final double P_LEFT = 0;
	public static final double I_LEFT = 0;
	public static final double D_LEFT = 0;
	
	//TODO: Find Right FPID values
	public static final double F_RIGHT = 0;//1023.0/(MAX_HIGH_GEAR_SPEED.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.HUNDRED_MILLISECOND) * NATIVE_UNITS_PER_REV);
	public static final double P_RIGHT = 0;
	public static final double I_RIGHT = 0;
	public static final double D_RIGHT = 0;

	public static enum DriveMode {
		arcadeDrive, tankDrive
	}
	
	public Speed currentMaxSpeed() {
		return MAX_SPEED;
	}
	
	public static final Distance PROFILE_POSITION_THRESHOLD = new Distance(1, Distance.Unit.INCH);
	
	public static final Time PROFILE_TIME_THRESHOLD = new Time(200, Time.Unit.MILLISECOND);
	
	private Drive() {
		if (EnabledSubsystems.DRIVE_ENABLED) {
			
			leftTalon = TalonCreator.createMasterTalon(RobotMap.DRIVE_LEFT);
			
			if (EnabledSubsystems.DRIVE_DUMB_ENABLED) {
				leftTalon.changeControlMode(TalonControlMode.PercentVbus);
			} else {
				leftTalon.changeControlMode(TalonControlMode.Speed);
			}
			leftTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder); //TODO: Make sure encoder type is correct
			leftTalon.setF(F_LEFT);
			leftTalon.setP(P_LEFT);
			leftTalon.setI(I_LEFT);
			leftTalon.setD(D_LEFT);
			leftTalon.configEncoderCodesPerRev(TICKS_PER_REV);
			leftTalon.enableBrakeMode(true);
			leftTalon.setEncPosition(0);
			leftTalon.reverseSensor(false);
			leftTalon.enable();
			
			leftEncoder = new TalonEncoder(leftTalon);
			
			tempLeftTalon.changeControlMode(TalonControlMode.Follower);
			tempLeftTalon.set(leftTalon.getDeviceID());
			tempLeftTalon.enableBrakeMode(true);
			
			rightTalon = TalonCreator.createMasterTalon(RobotMap.DRIVE_RIGHT);
			
			if (EnabledSubsystems.DRIVE_DUMB_ENABLED) {
				rightTalon.changeControlMode(TalonControlMode.PercentVbus);
			} else {
				rightTalon.changeControlMode(TalonControlMode.Speed);
			}
			rightTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder); //TODO: Make sure encoder type is correct
			rightTalon.setF(F_RIGHT);
			rightTalon.setP(P_RIGHT);
			rightTalon.setI(I_RIGHT);
			rightTalon.setD(D_RIGHT);
			rightTalon.configEncoderCodesPerRev(TICKS_PER_REV);
			rightTalon.enableBrakeMode(true);
			rightTalon.setEncPosition(0);
			rightTalon.reverseSensor(false);
			rightTalon.setInverted(false);
			rightTalon.enable();
			
			rightEncoder = new TalonEncoder(rightTalon);
			
			tempLeftTalon = TalonCreator.createFollowerTalon(RobotMap.TEMP_LEFT_TALON, leftTalon.getDeviceID());
			tempRightTalon = TalonCreator.createFollowerTalon(RobotMap.TEMP_RIGHT_TALON, rightTalon.getDeviceID());
			
			SmartDashboard.putNumber("Drive Percent", 0);
			SmartDashboard.putNumber("Distance to Profile in Feet", 0);
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
	
	public Distance getLeftDistance() {
		if(leftTalon != null) {
		return new Distance(leftTalon.getPosition(), Distance.Unit.DRIVE_ROTATION);
		}else
			return Distance.ZERO;
	}
	
	public Distance getRightDistance() {
		if(rightTalon != null) {
			return new Distance(rightTalon.getPosition(), Distance.Unit.DRIVE_ROTATION);
			}
		else {
			return Distance.ZERO;
		}
	}
	
	public void setMotorSpeedInPercent(double left, double right) {
		setMotorSpeed(currentMaxSpeed().mul(left), currentMaxSpeed().mul(right));
	}
	
	public void setMotorSpeed(Speed left, Speed right) {
		if (leftTalon != null && rightTalon != null) {
			
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
			}else {
				leftMotorSetpoint = left;
				rightMotorSetpoint = right.negate();
			}
			
			leftTalon.setF(((VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT * leftMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_LEFT) * 1023.0) / (new AngularSpeed(leftMotorSetpoint.abs().get(Distance.Unit.DRIVE_ROTATION, Time.Unit.HUNDRED_MILLISECOND), Angle.Unit.ROTATION, Time.Unit.HUNDRED_MILLISECOND).get(Angle.Unit.MAGNETIC_ENCODER_NATIVE_UNITS, Time.Unit.HUNDRED_MILLISECOND) / 4));
			rightTalon.setF(((VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT * rightMotorSetpoint.abs().get(Distance.Unit.FOOT, Time.Unit.SECOND) + MIN_MOVE_VOLTAGE_PERCENT_RIGHT) * 1023.0) / (new AngularSpeed(rightMotorSetpoint.abs().get(Distance.Unit.DRIVE_ROTATION, Time.Unit.HUNDRED_MILLISECOND), Angle.Unit.ROTATION, Time.Unit.HUNDRED_MILLISECOND).get(Angle.Unit.MAGNETIC_ENCODER_NATIVE_UNITS, Time.Unit.HUNDRED_MILLISECOND) / 4));
			
			if (leftTalon.getControlMode() == TalonControlMode.PercentVbus) {
				leftTalon.set(leftMotorSetpoint.div(currentMaxSpeed()));
			} else {
				leftTalon.set(leftMotorSetpoint.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE));
			}
			if (rightTalon.getControlMode() == TalonControlMode.PercentVbus) {
				rightTalon.set(rightMotorSetpoint.div(currentMaxSpeed()));
			} else {
				rightTalon.set(rightMotorSetpoint.get(Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE));
			}
		}
	}
	
	public Distance getLeftPosition() {
		if (leftTalon != null)
			return new Distance(leftTalon.getPosition(), Distance.Unit.DRIVE_ROTATION);
		return Distance.ZERO;
	}
	
	public Distance getRightPosition() {
		if (rightTalon != null)
			return new Distance(rightTalon.getPosition(), Distance.Unit.DRIVE_ROTATION);
		return Distance.ZERO;
	}
	
	public Distance getHistoricalLeftPosition(Time deltaTime) {
		if (leftEncoder != null)
			return new Distance(leftEncoder.getPosition(deltaTime), Distance.Unit.DRIVE_ROTATION);
		return Distance.ZERO;
	}
	
	public Distance getHistoricalRightPosition(Time deltaTime) {
		if (rightEncoder != null)
			return new Distance(rightEncoder.getPosition(deltaTime), Distance.Unit.DRIVE_ROTATION);
		return Distance.ZERO;
	}
	
	public Speed getLeftSpeed() {
		if (leftTalon != null)
			return new Speed(leftTalon.getSpeed(), Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE);
		return Speed.ZERO;
	}
	
	public Speed getRightSpeed() {
		if (rightTalon != null)
			return new Speed(rightTalon.getSpeed(), Distance.Unit.DRIVE_ROTATION, Time.Unit.MINUTE);
		return Speed.ZERO;
	}
	
	public Speed getHistoricalLeftSpeed(Time deltaTime) {
		if (leftEncoder != null)
			return new Speed(leftEncoder.getVelocity(deltaTime));
		return Speed.ZERO;
	}
	
	public Speed getHistoricalRightSpeed(Time deltaTime) {
		if (rightEncoder != null)
			return new Speed(rightEncoder.getVelocity(deltaTime));
		return Speed.ZERO;
	}
	
	public void setPID(double p, double i, double d, double f) {
		if (leftTalon != null)
			leftTalon.setPID(p, i, d, f, 0, 0, 0);
		if (rightTalon != null)
			rightTalon.setPID(p, i, d, f, 0, 0, 0);
	}
	
	private void setProfile(int profile) {
		if (leftTalon != null)
			leftTalon.setProfile(profile);
		if (rightTalon != null)
			rightTalon.setProfile(profile);
	}
	
	public void startDumbDrive() {
		if (leftTalon != null && rightTalon != null) {
			if (rightTalon.getControlMode() != TalonControlMode.PercentVbus) {
				rightTalon.changeControlMode(TalonControlMode.PercentVbus);
			}
			if (leftTalon.getControlMode() != TalonControlMode.PercentVbus) {
				leftTalon.changeControlMode(TalonControlMode.PercentVbus);
			}
		}
	}
	
	public void endDumbDrive() {
		if (leftTalon != null && rightTalon != null) {
			if (rightTalon.getControlMode() != TalonControlMode.Speed) {
				rightTalon.changeControlMode(TalonControlMode.Speed);
			}
			if (leftTalon.getControlMode() != TalonControlMode.Speed) {
				leftTalon.changeControlMode(TalonControlMode.Speed);
			}
		}
	}
	
	@Override
	public void periodic() {

	}
	
	@Override
	public void smartDashboardInfo() {
		if (leftTalon != null && rightTalon != null) {
			if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_BASIC_ENABLED) {
				SmartDashboard.putString("Drive Current", getLeftCurrent() + " : " + getRightCurrent());
				//SmartDashboard.putString("Drive Left Speed", getLeftSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
				//SmartDashboard.putString("Drive Right Speed", getRightSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
				if (Gyro.chosenGyro.equals(ChosenGyro.NavX)) {
					SmartDashboard.putNumber("Gyro Yaw", NavX.getInstance().getYaw().get(Angle.Unit.DEGREE));
				} else {
					SmartDashboard.putNumber("Gyro Yaw", Pigeon.getInstance().getYaw().get(Angle.Unit.DEGREE));
				}
				SmartDashboard.putNumber("Drive Left Percent", leftMotorSetpoint.div(currentMaxSpeed()));
				SmartDashboard.putNumber("Drive Right Percent", rightMotorSetpoint.div(currentMaxSpeed()));
				
				SmartDashboard.putString("Drive Left Velocity vs Set Velocity: ", getLeftSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
				SmartDashboard.putString("Drive Right Velocity vs Set Velocity: ", getRightSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
				
			}
			if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_COMPLEX_ENABLED) {
				SmartDashboard.putData(this);
				SmartDashboard.putString("Drive Voltage",
						leftTalon.getOutputVoltage() + " : " + rightTalon.getOutputVoltage());
				SmartDashboard.putNumber("Drive Left Position", getLeftPosition().get(Distance.Unit.FOOT));
				SmartDashboard.putNumber("Drive Right Position", getRightPosition().get(Distance.Unit.FOOT));
			}
		/*	SmartDashboard.putNumber("Drive Left Current", leftDrive.getOutputCurrent());
			SmartDashboard.putNumber("Drive Right Current", rightDrive.getOutputCurrent());
			
			SmartDashboard.putString("Drive Left Speed vs Set Speed: ", getLeftSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
			SmartDashboard.putString("Drive Right Speed vs Set Speed: ", -getRightSpeed().get(Distance.Unit.FOOT, Time.Unit.SECOND) + " : " + -rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND));
			
			SmartDashboard.putNumber("Drive Left Voltage", leftDrive.getOutputVoltage());
			SmartDashboard.putNumber("Drive Right Voltage", rightDrive.getOutputVoltage());
		
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
		*/
		}
	}
	
	@Override
	public void disable() {
		setMotorSpeedInPercent(0, 0);
	}
	
	public double getRightCurrent() {
		if (rightTalon != null) {
			return rightTalon.getOutputCurrent();
		}
		return 0;
	}
	
	public double getLeftCurrent() {
		if (leftTalon != null) {
			return leftTalon.getOutputCurrent();
		}
		return 0;
	}
	
	public void setVoltageRampRate(double rampRate) {
		if(rightTalon != null) {
			rightTalon.setVoltageRampRate(rampRate);
		} 
		if(tempRightTalon != null) {
			tempRightTalon.setVoltageRampRate(rampRate);
		}
		if(leftTalon != null) {
			leftTalon.setVoltageRampRate(rampRate);
		}
		if(tempLeftTalon != null) {
			tempLeftTalon.setVoltageRampRate(rampRate);
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
			return getInstance().getLeftSpeed().get(Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND);
		} else {
			return getInstance().getLeftPosition().get(Distance.Unit.DRIVE_ROTATION);
		}
	}
	
	@Override
	public double pidGetRight() {
		if (type == PIDSourceType.kRate) {
			return -getInstance().getRightSpeed().get(Distance.Unit.DRIVE_ROTATION, Time.Unit.SECOND);
		} else {
			return -getInstance().getRightPosition().get(Distance.Unit.DRIVE_ROTATION);
		}
	}
	
	@Override
	public void pidWrite(double outputLeft, double outputRight) {
		setMotorSpeed(currentMaxSpeed().mul(outputLeft),currentMaxSpeed().mul(outputRight));
	}
}
