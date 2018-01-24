package edu.nr.robotics.subsystems.sensors;

import edu.nr.lib.commandbased.NRCommand;

public class EnableFloorSensorCommand extends NRCommand {
	
	private boolean bool;
	
	public EnableFloorSensorCommand(boolean bool) {
		this.bool = bool;
	}

	@Override
	protected void onStart() {
		EnabledSensors.floorTapeSeen = false;
		EnabledSensors.floorSensorEnabled = bool;
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
	
}
