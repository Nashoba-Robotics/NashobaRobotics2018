package edu.nr.robotics.subsystems.sensors;

import edu.nr.lib.commandbased.NRCommand;

public class EnablePortalSensorCommand extends NRCommand {

	private boolean bool;
	
	public EnablePortalSensorCommand(boolean bool) {
		this.bool = bool;
	}

	@Override
	protected void onStart() {
		EnabledSensors.portalSensorEnabled = bool;
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
