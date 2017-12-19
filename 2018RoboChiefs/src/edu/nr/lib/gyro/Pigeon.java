package edu.nr.lib.gyro;

import com.kauailabs.sf2.orientation.OrientationHistory;
import com.ctre.CANTalon;
import com.ctre.PigeonImu;

import edu.nr.lib.interfaces.Periodic;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Time;

public class Pigeon extends Gyro implements Periodic {

	private static Pigeon singleton;
	
	private PigeonImu pigeon;
	private CANTalon talon;
	private int talonID = 3;
	
	private double[] yawPitchRoll;
	
	OrientationHistory orientationHistory;
	
	double lastWriteTimestamp = 0;
	
	public static Pigeon getInstance() {
		init();
		return singleton;
	}
	
	public synchronized static void init() {
		if(singleton == null) {
			System.out.println("PigeonImu talonID not specified");
			singleton = new Pigeon();
		}
	}
	
	public Pigeon() {
		talon = new CANTalon(talonID);
		pigeon = new PigeonImu(talon);
		
		yawPitchRoll = new double[3];
		
		try {
			periodics.add(this);
		} catch (Exception ex) {
			System.out.println("Error instantiating Pigeon IMU");
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
		pigeon.GetYawPitchRoll(yawPitchRoll);
		return new Angle(yawPitchRoll[0], Angle.Unit.DEGREE);
	}
	
	@Override
	public void periodic() {
	}

	@Override
	public void reset() {
		pigeon.SetYaw(0);
	}
	
}
