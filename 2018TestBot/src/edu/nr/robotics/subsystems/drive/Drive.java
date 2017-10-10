package edu.nr.robotics.subsystems.drive;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;
import edu.nr.lib.sensorhistory.TalonEncoder;
import edu.nr.robotics.RobotMap;
import edu.nr.robotics.subsystems.EnabledSybsystems;
import edu.wpi.first.wpilibj.PIDSourceType;

public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {
	
	private static Drive singleton;
	
	public static final int WHEEL_DIAMETER_INCHES = 1; //not correct TODO: get actual value from thing
	public static final double MAX_SPEED = 0; //TODO: Find for real
	
	private CANTalon leftDrive, rightDrive, rightDriveFollow, leftDriveFollow;
	private TalonEncoder leftEncoder, RightEncoder;
	
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
	
	public static enum DriveMode {
		tankDrive, arcadeDrive
	}
	
	private Drive() {
		
		if(EnabledSybsystems.DRIVE_ENABLED) {
			
			leftDrive = new CANTalon(RobotMap.DRIVE_LEFT);
			rightDrive = new CANTalon(RobotMap.DRIVE_RIGHT);
			
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
			
			leftEncoder = new TalonEncoder(leftDrive);
			
			leftDriveFollow = new CANTalon(RobotMap.DRIVE_LEFT_FOLLOW);
			rightDriveFollow = new CANTalon(RobotMap.DRIVE_RIGHT_FOLLOW);
			
			leftDriveFollow.changeControlMode(TalonControlMode.Follower);
			leftDriveFollow.set(leftDrive.getDeviceID());
			leftDriveFollow.enableBrakeMode(true);
			
			rightDriveFollow.changeControlMode(TalonControlMode.Follower);
			rightDriveFollow.set(rightDrive.getDeviceID());
			rightDriveFollow.enableBrakeMode(true);
			
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

	public void pidWrite(double outputLeft, double outputRight) {
		
	}

	public void smartDashboardInfo() {
		
	}

	public void periodic() {
		
	}

	public void disable() {
		
	}
	
	

}
