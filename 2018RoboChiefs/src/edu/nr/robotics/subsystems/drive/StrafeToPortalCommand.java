package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class StrafeToPortalCommand extends NRCommand {

	Direction direction;
	
	public StrafeToPortalCommand(Direction direction) {
		this.direction = direction;
	}
	
	@Override
	protected void onStart() {
		if (direction == Direction.left) {
			Drive.getInstance().arcadeDrive(0, 0, -Drive.PORTAL_STRAFE_PERCENT);
		} else {
			Drive.getInstance().arcadeDrive(0, 0, Drive.PORTAL_STRAFE_PERCENT);
		}
	}
	
	@Override
	protected void onEnd() {
		Drive.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return EnabledSensors.portalSensorLeft.get() && EnabledSensors.portalSensorRight.get();
	}

}
