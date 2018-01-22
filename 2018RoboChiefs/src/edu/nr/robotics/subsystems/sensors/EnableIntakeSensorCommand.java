package edu.nr.robotics.subsystems.sensors;

import edu.nr.lib.commandbased.NRCommand;

public class EnableIntakeSensorCommand extends NRCommand {

	private boolean bool;
	
	public EnableIntakeSensorCommand(boolean bool) {
		this.bool = bool;
	}
	
	@Override
	protected void onStart() {
		EnabledSensors.intakeSensorEnabled = bool;
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
