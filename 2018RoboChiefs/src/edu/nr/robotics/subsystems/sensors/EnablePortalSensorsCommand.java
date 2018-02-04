package edu.nr.robotics.subsystems.sensors;

import edu.nr.lib.commandbased.NRCommand;

public class EnablePortalSensorsCommand extends NRCommand {

	private boolean bool;
	
	public EnablePortalSensorsCommand(boolean bool) {
		this.bool = bool;
	}
	
	@Override
	protected void onStart() {
		EnabledSensors.portalSensorEnabled = bool;
		EnabledSensors.portalReached = false;
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
