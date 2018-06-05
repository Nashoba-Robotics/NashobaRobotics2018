package edu.nr.robotics.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.nr.lib.interfaces.TriplePIDOutput;
import edu.nr.lib.interfaces.TriplePIDSource;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Speed;
import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Drive extends Subsystem implements TriplePIDOutput, TriplePIDSource {

	private static Drive singleton;
	
	private TalonSRX leftDrive, rightDrive, leftDriveFollow, rightDriveFollow;
	
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
	
	
	
}
