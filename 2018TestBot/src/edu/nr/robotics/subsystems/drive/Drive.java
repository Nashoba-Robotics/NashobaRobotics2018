package edu.nr.robotics.subsystems.drive;

import com.ctre.CANTalon;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.interfaces.DoublePIDOutput;
import edu.nr.lib.interfaces.DoublePIDSource;
import edu.nr.lib.sensorhistory.TalonEncoder;
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
