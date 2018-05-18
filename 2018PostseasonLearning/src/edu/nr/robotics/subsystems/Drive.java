package edu.nr.robotics.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.nr.robotics.RobotMap;
import edu.wpi.first.wpilibj.PIDSourceType;

public class Drive extends NRSubsystem implements DoublePIDOutput, DoublePIDSource {

	private static Drive singleton;
	
	private TalonSRX leftDrive, rightDrive, leftDriveFollow, rightDriveFollow, pigeonTalon ;
	
	public static final double REAL_ENC_TICK_PER_INCH_DRIVE = 0; //TODO: Find encoder ticks per inch drive
	
	
	public static final double EFFECTIVE_ENC_TICK_PER_INCH_DRIVE = REAL_ENC_TICK_PER_INCH_DRIVE;
	
	public static final Speed MAX_SPEED_DRIVE = new Speed(0, Distance.Unit.FOOT, Time.Unit.SECOND);
	
	public static final Acceleration MAX_ACCEL_DRIVE = new Acceleration(0, Distance.Unit.FOOT, Time.Unit.SECOND, Time.Unit.SECOND); //TODO: Find max accel drive 
	
	public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0; //TODO: find minimum move voltage percent left
	public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0; //TODO: find minimum move voltage percent right
	
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0.0; //TODO: Find these
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0.0;
	
	public static Time DRIVE_RAMP_RATE = new Time(0.0, Time.Unit.SECOND);  //TODO: Find this
	
	
	public static double P_LEFT = 0;   //TODO: Find all of these
	public static double I_LEFT = 0;
	public static double D_LEFT = 0;
		
	public static double P_RIGHT = 0;
	public static double I_RIGHT = 0;
	public static double D_RIGHT = 0;
	
	
	public static double kVOneD = 1 / MAX_SPEED_DRIVE.get(Distance.Unit.MAGNETIC_ENCODER_TICK_DRIVE, Time.Unit.HUNDRED_MILLISECOND); //TODO: Find this stuff as well
	public static double kAOneD = 0.0;
	public static double kPOneD = 0;
	public static double kIOneD = 0;
	public static double kDOneD = 0;
	public static double kP_thetaOneD = 0.0;
	
	
	public static final double PROFILE_DRIVE_PERCENT = 0.8; //TODO: Find this one too
	
	public static final double ACCEL_PERCENT = 0.8; //TODO: Find this
	
	public static double TURN_JOYSTICK_MULTIPLIER = 1.0; 
	

	public static final double MAX_PROFILE_TURN_PERCENT = 1.0;  //TODO: Find this
	public static final double MIN_PROFILE_TURN_PERCENT = 0.02; 
	
	public static final Distance END_THRESHOLD = new Distance(3, Distance.Unit.INCH);
	
	public static final Speed PROFILE_END_TURN_SPEED_THRESHOLD = MAX_SPEED_DRIVE.mul(MIN_PROFILE_TURN_PERCENT + 0.01);
	
	private static final int PID_TYPE = 0;
	private static final int DEFAULT_TIMEOUT = 0;
	private static final int VEL_SLOT = 0;
	
	private Drive(){
		if(EnabledSubsystems.DRIVE_ENABLED){
			
			leftDrive = CTRECreator.createMasterTalon(RobotMap.LEFT_DRIVE_MASTER_TALON);
			rightDrive = CTRECreator.createMasterTalon(RobotMap.RIGHT_DRIVE_MASTER_TALON);
			
			leftDriveFollow = CTRECreator.createFollowerTalon(RobotMap.LEFT_DRIVE_FOLLOWER_TALON, RobotMap.LEFT_DRIVE_MASTER_TALON );
			rightDriveFollow = CTRECreator.createFollowerTalon(RobotMap.RIGHT_DRIVE_FOLLOWER_TALON, RobotMap.RIGHT_DRIVE_MASTER_TALON);
			
			leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_TYPE, DEFAULT_TIMEOUT);
			leftDrive.configkF(VEL_SLOT, 0, DEFAULT_TIMEOUT)
		}
	}
	
	public static void init(){
		if(singleton == null){
			singleton = new Drive();
		}
	}
	
	public static Drive getInstance() {
		init();
		return singleton;
	}

	@Override
	public void smartDashboardInfo() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double pidGetLeft() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double pidGetRight() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void pidWrite(double outputLeft, double outputRight) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void disable() {
		// TODO Auto-generated method stub
		
	}
	
}
