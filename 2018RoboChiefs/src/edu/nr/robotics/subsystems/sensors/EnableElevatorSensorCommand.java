package edu.nr.robotics.subsystems.sensors;

import edu.nr.lib.commandbased.NRCommand;

public class EnableElevatorSensorCommand extends NRCommand {

	private boolean bool;
	
	public EnableElevatorSensorCommand(boolean bool) {
		this.bool = bool;
	}
	
	@Override
	protected void onStart() {
		EnabledSensors.elevatorSensorEnabled = bool;
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
}
