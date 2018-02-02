package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;

public class StrafeToCubeCommand extends NRCommand {

	private Direction direction;
	
	public StrafeToCubeCommand(Direction direction) {
		this.direction = direction;
	}
	
	@Override
	protected void onStart() {
		new EnableLimelightCommand(true);
		if (direction == Direction.left) {
			Drive.getInstance().arcadeDrive(0, 0, -Drive.SENSOR_STRAFE_PERCENT);
		} else {
			Drive.getInstance().arcadeDrive(0, 0, Drive.SENSOR_STRAFE_PERCENT);
		}
	}
	
	@Override
	protected void onEnd() {
		Drive.getInstance().disable();
	}

	@Override
	protected boolean isFinishedNR() {
		return LimelightNetworkTable.getInstance().getHorizOffset().lessThan(Drive.DRIVE_ANGLE_THRESHOLD);
	}
}
