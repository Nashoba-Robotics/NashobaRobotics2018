package edu.nr.robotics.subsystems.sensors;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.drive.Drive;

public class EnableFloorSensorCommand extends NRCommand {
	
	private boolean bool;
	
	public EnableFloorSensorCommand(boolean bool) {
		this.bool = bool;
	}

	@Override
	protected void onStart() {
		EnabledSensors.floorSensorEnabled = bool;
		EnabledSensors.floorCounter.reset();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
	
}
