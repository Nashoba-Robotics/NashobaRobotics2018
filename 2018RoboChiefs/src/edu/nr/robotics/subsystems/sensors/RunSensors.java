package edu.nr.robotics.subsystems.sensors;

import java.util.TimerTask;
import java.util.Timer;

public class RunSensors extends TimerTask {

	private final Timer timer;
	private final static long defaultPeriod = 1; //ms
	
	public RunSensors() {
		timer = new Timer();
		timer.scheduleAtFixedRate(this, 0, defaultPeriod);
	}

	@Override
	public void run() {
		
		if (EnabledSensors.floorSensorEnabled) {
			if (!EnabledSensors.floorTapeSeen && !EnabledSensors.floorSensor.get()) {
				EnabledSensors.floorTapeSeen = true;
			}
		}
	}
}
