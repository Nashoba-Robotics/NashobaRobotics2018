package edu.nr.lib.gyro;

import com.kauailabs.sf2.orientation.OrientationHistory;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.nr.lib.interfaces.Periodic;
import edu.nr.lib.talons.CTRECreator;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.drive.Drive;

public class Pigeon extends Gyro implements Periodic {

	private static Pigeon singleton;
	
	private PigeonIMU pigeon;
	private static int talonID = 6; //intake rollers
	
	private double[] yawPitchRoll;
	private short[] XYZError;
	
	OrientationHistory orientationHistory;
	
	double lastWriteTimestamp = 0;
	
	public Pigeon(TalonSRX talon) {
		pigeon = CTRECreator.createPigeon(talon);
		talonID = talon.getDeviceID();
		
		yawPitchRoll = new double[3];
		
		try {
			periodics.add(this);
		} catch (Exception ex) {
			System.out.println("Error instantiating Pigeon IMU");
		}
	}
	
	public static Pigeon getPigeon(TalonSRX talon) {
		if (singleton == null) {
			init(talon);
		}
		if (talon.getDeviceID() == talonID) {
			return singleton;
		} else {
			System.out.println("Pigeon doesn't exist");
			return null;
		}
	}
	
	public synchronized static void init(TalonSRX talon) {
		if(singleton == null) {
			singleton = new Pigeon(talon);
		}
	}
	
	/**
	 * This is not yet a working function since no Pigeon logging has been created
	 */
	public Angle getHistoricalYaw(Time deltaTime) {
		return Angle.ZERO;
	}
	
	@Override
	public Angle getYaw() {
		pigeon.getYawPitchRoll(yawPitchRoll);
		return new Angle(yawPitchRoll[0], Angle.Unit.DEGREE);
	}
	
	@Override
	public void periodic() {
	}

	@Override
	public void reset() {
		pigeon.setYaw(0, 0);
	}

	@Override
	public Distance getYError() {
		pigeon.getBiasedAccelerometer(XYZError);
		return new Distance(XYZError[0], Distance.Unit.FOOT); //THIS IS WRONG PROBABLY IN MANY WAYS
		
	}
	
}
