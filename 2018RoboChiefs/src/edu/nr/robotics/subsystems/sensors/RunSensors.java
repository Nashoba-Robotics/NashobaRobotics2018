package edu.nr.robotics.subsystems.sensors;

import java.util.Timer;
import java.util.TimerTask;

public class RunSensors extends TimerTask {

	private final Timer timer;
	private final static long defaultPeriod = 5; //ms
	
	public RunSensors() {
		timer = new Timer();
		timer.scheduleAtFixedRate(this, 0, defaultPeriod);
	}
	
	@Override
	public void run() {
		
		//System.out.println("1: " + EnabledSensors.portalSensorLeft1.get() + " 2: " + EnabledSensors.portalSensorLeft2.get() +
		//		" 3: " + EnabledSensors.portalSensorRight1.get() + " 4: " + EnabledSensors.portalSensorRight2.get());
		
		if(EnabledSensors.portalSensorEnabled) {
			if (!EnabledSensors.portalReached && EnabledSensors.portalSensorLeft1.get() && EnabledSensors.portalSensorLeft2.get()
					&& EnabledSensors.portalSensorRight1.get() && EnabledSensors.portalSensorRight2.get()) {
				EnabledSensors.portalReached = true;
			}
		}
	}
}
