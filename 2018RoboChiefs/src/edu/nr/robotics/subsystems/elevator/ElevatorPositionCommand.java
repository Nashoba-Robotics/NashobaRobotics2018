package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorPositionCommand extends NRCommand {

	private Distance height;
	
	public ElevatorPositionCommand(Distance height) {
		this.height = height;
	}
	
	@Override
	protected void onStart() {
		Elevator.getInstance().setPosition(height);
	}
	
	@Override
	protected boolean isFinishedNR() {
		boolean finished = (Elevator.getInstance().getHistoricalPosition(Elevator.PROFILE_DELTA_TIME_THRESHOLD_ELEVATOR)
		.sub(Elevator.getInstance().getPosition())).abs().lessThan(Elevator.PROFILE_DELTA_POS_THRESHOLD_ELEVATOR)
		&& (Elevator.getInstance().getHistoricalPosition(Elevator.PROFILE_DELTA_TIME_THRESHOLD_ELEVATOR.mul(2))
				.sub(Elevator.getInstance().getPosition())).abs().lessThan(Elevator.PROFILE_DELTA_POS_THRESHOLD_ELEVATOR)
		&& (Elevator.getInstance().getPosition().sub(height)).abs().lessThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR);
		return finished;
	}
}
