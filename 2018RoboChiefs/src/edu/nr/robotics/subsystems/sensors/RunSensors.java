package edu.nr.robotics.subsystems.sensors;

import java.util.Timer;
import java.util.TimerTask;

public class RunSensors extends TimerTask {
	
	private final Timer timer;
	private final static long defaultPeriod = 1; //ms

	public RunSensors() {
		timer = new Timer();
		timer.scheduleAtFixedRate(this, 0, defaultPeriod);
	}
	
	@Override
	public void run() {

		if (EnabledSensors.portalSensorEnabled) {
			if (!EnabledSensors.portalReached && EnabledSensors.portalSensorLeft1.get() && EnabledSensors.portalSensorLeft2.get()
					&& EnabledSensors.portalSensorRight1.get() && EnabledSensors.portalSensorRight2.get()) {

				EnabledSensors.portalReached = true;
				
			}
		}
		
	}

}
