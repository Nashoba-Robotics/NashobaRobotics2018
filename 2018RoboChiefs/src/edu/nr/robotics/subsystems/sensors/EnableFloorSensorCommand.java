package edu.nr.robotics.subsystems.sensors;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.drive.Drive;

public class EnableFloorSensorCommand extends NRCommand {
	
	private static final Time FLOOR_SENSOR_RAMP_RATE = Time.ZERO; //TODO: find the floor sensor ramp rate
	
	private boolean bool;
	
	public EnableFloorSensorCommand(boolean bool) {
		this.bool = bool;
	}

	@Override
	protected void onStart() {
		EnabledSensors.floorTapeSeen = false;
		EnabledSensors.floorSensorEnabled = bool;
		if (bool) {
			Drive.getInstance().SetVoltageRamp(FLOOR_SENSOR_RAMP_RATE);
		} else {
			Drive.getInstance().SetVoltageRamp(Drive.DRIVE_RAMP_RATE);
		}
	}
	
	@Override
	protected boolean isFinishedNR() {
		return true;
	}
	
}
