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
import edu.nr.lib.gyro.ResetGyroCommand;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;
import edu.nr.lib.motionprofiling.OneDimensionalMotionProfilerTwoMotor;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Distance.Unit;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {

	private static Drive singleton;
	
	private TalonSRX leftDrive, rightDrive, leftDriveFollow, rightDriveFollow, pigeonTalon;
	
	public static final double ENC_TO_WHEEL_GEARING = 1.0;
	
	public static final double EFFECTIVE_WHEEL_DIAMETER_INCHES = 0.0 * 0.958333333333333;//TODO find
	
	public static final int ENC_TICK_PER_REVOLUTION = 256;
	
	//public static final double EFFECTIVE_ENC_TICK_PER_INCH_DRIVE = ENC_TICK_PER_REVOLUTION / (Math.PI * EFFECTIVE_WHEEL_DIAMETER_INCHES);
	public static final double EFFECTIVE_ENC_TICK_PER_INCH_DRIVE = 428;
	
	
	public static final Distance REAL_WHEEL_DIAMETER = new Distance(EFFECTIVE_WHEEL_DIAMETER_INCHES, Distance.Unit.INCH);
	
	public static final Distance EFFECTIVE_WHEEL_DIAMETER = new Distance(EFFECTIVE_WHEEL_DIAMETER_INCHES / ENC_TO_WHEEL_GEARING, Distance.Unit.INCH);
	
	public static final Speed MAX_SPEED_DRIVE = new Speed(12.3, Distance.Unit.FOOT, Time.Unit.SECOND);
	
	public static final Acceleration MAX_ACCEL_DRIVE = new Acceleration(0.0, Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND);
	
	public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0.0982; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0.107; //This is 0 to 1 number
	
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0.0733;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0.0707;
	
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
	
	private static final int PEAK_DRIVE_CURRENT = 60;
	private static final int PEAK_DRIVE_CURRENT_DURATION = 1000;
	private static final int CONTINUOUS_CURRENT_LIMIT = 40;
	
	public static final VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD_DRIVE = VelocityMeasPeriod.Period_10Ms;
	public static final int VELOCITY_MEASUREMENT_WINDOW_DRIVE = 0;
	
	private static final int VOLTAGE_COMPENSATION_LEVEL = 12;
	
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
	public static double accelPercent;
	public static double drivePercent;
	public static Angle angleToTurn; 
	public static boolean exact;
	
	public static enum DriveMode {
		arcadeDrive, tankDrive, cheesyDrive
	}
	
	private OneDimensionalMotionProfilerTwoMotor Profiler;
	
	private Drive() {
	//	System.out.println("Test");
		if(EnabledSubsystems.DRIVE_ENABLED) {
			
			leftDrive = CTRECreator.createMasterTalon(RobotMap.LEFT_DRIVE_MASTER_TALON);
			rightDrive = CTRECreator.createMasterTalon(RobotMap.RIGHT_DRIVE_MASTER_TALON);
			
			leftDriveFollow = CTRECreator.createFollowerTalon(RobotMap.LEFT_DRIVE_FOLLOWER_TALON, RobotMap.LEFT_DRIVE_MASTER_TALON );
			rightDriveFollow = CTRECreator.createFollowerTalon(RobotMap.RIGHT_DRIVE_FOLLOWER_TALON, RobotMap.RIGHT_DRIVE_MASTER_TALON);
			
			pigeonTalon = CTRECreator.createMasterTalon(RobotMap.PIGEON_TALON);
			
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
			
			CheesyDriveCalculationConstants.createDriveTypeCalculations();
			
			smartDashboardInit();
		}
	}
			public void smartDashboardInit(){
				
				//System.out.println("sd init");
				if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_BASIC_ENABLED) {
					SmartDashboard.putData(new ResetGyroCommand());
				}
				if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
					
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
					
					SmartDashboard.putNumber("Drive Ramp Rate: ", DRIVE_RAMP_RATE.get(Time.Unit.SECOND));
					
					SmartDashboard.putNumber("X Profile Feet: ", 0);
					SmartDashboard.putNumber("Y Profile Feet: ", 0);
				//	System.out.println("is this being called");
					SmartDashboard.putNumber("Drive Percent: ", PROFILE_DRIVE_PERCENT);
					SmartDashboard.putNumber("Drive Accel Percent: ", ACCEL_PERCENT);
					SmartDashboard.putNumber("Angle To Turn: ", 0);
					
				}
			}
	
	public void smartDashboardInfo() {
		
	//	System.out.println("Tyler said to do this7");
		if (leftDrive != null && rightDrive != null) {
	//		System.out.println("Tyler said to do this8");
			if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_BASIC_ENABLED) {
	//			System.out.println("Tyler said to do this9");
								
				SmartDashboard.putNumberArray("Drive Left Current", new double [] { getLeftCurrent(), getLeftFollowCurrent()});
				
				SmartDashboard.putNumberArray("Drive Right Current", new double [] { getRightCurrent(), getRightFollowCurrent()} );
				
				
				if (Gyro.chosenGyro.equals(ChosenGyro.NavX)) {
					SmartDashboard.putNumber("Gyro Yaw", (-NavX.getInstance().getYaw().get(Angle.Unit.DEGREE)) % 360);
				} else {
					SmartDashboard.putNumber("Gyro Yaw", (-Pigeon.getPigeon(getInstance().getPigeonTalon()).getYaw().get(Angle.Unit.DEGREE)) % 360);
				}
				
				SmartDashboard.putNumberArray("Drive Left Velocity: ", new double [] {getLeftVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) ,leftMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND)});
				
				SmartDashboard.putNumberArray("Drive Right Velocity: ", new double [] {getRightVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND) , rightMotorSetpoint.get(Distance.Unit.FOOT, Time.Unit.SECOND)});
				
			
				
			}
			//System.out.println("Tyler said to do this6");
			if (EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
			//	System.out.println("Tyler said to do this5");
				SmartDashboard.putNumber("Drive Left Percent", leftMotorSetpoint.div(MAX_SPEED_DRIVE));
				SmartDashboard.putNumber("Drive Right Percent", rightMotorSetpoint.div(MAX_SPEED_DRIVE));
				
			//	System.out.println("Tyler said to do this4");
				SmartDashboard.putData(this);
				SmartDashboard.putNumberArray("Drive Voltage",
						new double [] {leftDrive.getMotorOutputVoltage(), rightDrive.getMotorOutputVoltage()});
				SmartDashboard.putNumber("Drive Left Position", getLeftPosition().get(Distance.Unit.INCH));
				SmartDashboard.putNumber("Drive Right Position", getRightPosition().get(Distance.Unit.INCH));
			//	System.out.println("Tyler said to do this3");
				SmartDashboard.putNumber("Drive Left Encoder Position", leftDrive.getSelectedSensorPosition(PID_TYPE));
				SmartDashboard.putNumber("Drive Right Encoder Position", rightDrive.getSelectedSensorPosition(PID_TYPE));
			//	System.out.println("Tyler said to do this2");
				leftDrive.config_kP(VEL_SLOT, SmartDashboard.getNumber("Left P Value: ", P_LEFT), DEFAULT_TIMEOUT);
				leftDrive.config_kI(VEL_SLOT, SmartDashboard.getNumber("Left I Value: ", I_LEFT), DEFAULT_TIMEOUT);
				leftDrive.config_kD(VEL_SLOT, SmartDashboard.getNumber("Left D Value: ", D_LEFT), DEFAULT_TIMEOUT);
			//	System.out.println("Tyler said to do this1");
				rightDrive.config_kP(VEL_SLOT, SmartDashboard.getNumber("Right P Value: ", P_RIGHT), DEFAULT_TIMEOUT);
				rightDrive.config_kI(VEL_SLOT, SmartDashboard.getNumber("Right I Value: ", I_RIGHT), DEFAULT_TIMEOUT);
				rightDrive.config_kD(VEL_SLOT, SmartDashboard.getNumber("Right D Value: ", D_RIGHT), DEFAULT_TIMEOUT);
				
	
				P_LEFT = SmartDashboard.getNumber("Left P Value: ", P_LEFT);
				I_LEFT = SmartDashboard.getNumber("Left I Value: ", I_LEFT);
				D_LEFT = SmartDashboard.getNumber("Left D Value: ", D_LEFT);
				
				P_RIGHT = SmartDashboard.getNumber("Right P Value: ", P_RIGHT);
				I_RIGHT = SmartDashboard.getNumber("Right I Value: ", I_RIGHT);
				D_RIGHT = SmartDashboard.getNumber("Right D Value: ", D_RIGHT);
				
				
				kVOneD = SmartDashboard.getNumber("kVOneD Value: ", kVOneD);
				kAOneD = SmartDashboard.getNumber("kAOneD Value: ", kAOneD);
				kPOneD = SmartDashboard.getNumber("kPOneD Value: ", kPOneD);
				kIOneD = SmartDashboard.getNumber("kIOneD Value: ", kIOneD);
				kDOneD = SmartDashboard.getNumber("kDOneD Value: ", kDOneD);
				kP_thetaOneD = SmartDashboard.getNumber("kP_thetaOneD Value: ", kP_thetaOneD);
				
				
				DRIVE_RAMP_RATE = new Time(SmartDashboard.getNumber("Drive Ramp Rate: ", DRIVE_RAMP_RATE.get(Time.Unit.SECOND)), Time.Unit.SECOND);

				xProfile = new Distance(SmartDashboard.getNumber("X Profile Feet: ", 0), Distance.Unit.FOOT);
				yProfile = new Distance(SmartDashboard.getNumber("Y Profile Feet: ", 0), Distance.Unit.FOOT);
				drivePercent = SmartDashboard.getNumber("Drive Percent: ", 0);
				accelPercent = SmartDashboard.getNumber("Drive Accel Percent: ", 0);
				angleToTurn = new Angle(SmartDashboard.getNumber("Angle To Turn: ", 0), Angle.Unit.DEGREE);
				
				
				
			}
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
	
	//@Override
	public void disable() {
		setMotorSpeedInPercent(0, 0);
	}
	
	public void setMotorSpeed(Speed left, Speed right) {
		System.out.println(left.get(Distance.Unit.FOOT, Time.Unit.SECOND) + "    Left");
		
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
	
	public void cheesyDrive(double move, double turn) {
		double[] cheesyMotorPercents = new double[2];
		cheesyMotorPercents = DriveTypeCalculations.cheesyDrive(move, turn, oldTurn, false);
		
		oldTurn = turn;
		
		
		tankDrive(cheesyMotorPercents[0], cheesyMotorPercents[1]);		
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
	
	//@Override
	public PIDSourceType getPIDSourceType() {
		return type;
	}
	

	//@Override
	public double pidGetLeft() {
		if (type == PIDSourceType.kRate) {
			return getInstance().getLeftVelocity().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
		} else {
			return getInstance().getLeftPosition().get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE);
		}
	}
	
	//@Override
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
	
	public void enableMotionProfiler(Distance distX, Distance distY, double maxVelPercent, double maxAccelPercent) {
		double minVel = 0;//TODO find
		double minAccel = 0;//TODO find
		
		if (distX.equals(Distance.ZERO) && !distY.equals(Distance.ZERO)) {
			
		} else if (distY.equals(Distance.ZERO) && !distX.equals(Distance.ZERO)) {
			minVel = MAX_SPEED_DRIVE.mul(maxVelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND);
			minAccel = MAX_ACCEL_DRIVE.mul(maxAccelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND);
		} else if (!distX.equals(Distance.ZERO) && !distY.equals(Distance.ZERO)) {
			minVel = Math.min((NRMath.hypot(distX, distY).div(distX)) * MAX_SPEED_DRIVE.mul(maxVelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND), (NRMath.hypot(distX, distY).div(distY)));
			minAccel = Math.min((NRMath.hypot(distX, distY).div(distX)) * MAX_ACCEL_DRIVE.mul(maxAccelPercent).get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND, Time.Unit.HUNDRED_MILLISECOND), (NRMath.hypot(distX, distY).div(distY)));
		} else {
			minVel = 0;
			minAccel = 0;
			System.out.println("No Distances Set");
		}
				
		Profiler = new OneDimensionalMotionProfilerTwoMotor(this, this, kVOneD, kAOneD, kPOneD, kIOneD, kDOneD, kP_thetaOneD);
		Profiler.setTrajectory(new OneDimensionalTrajectoryRamped(distX.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE), minVel, minAccel, 1)); //TODO find the thing that is one. timeMult
		Profiler.enable();
}
	
	public void disableProfiler() {
		Profiler.disable();
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