package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorDeltaPositionCommand extends NRCommand{

	private Distance initialPos;
	private Distance deltaHeight;
	
	/**
	 * Sets the elevator position to the change specified
	 * @param deltaHeight: The change in height to go. Up is positive and down is negative
	 */
	public ElevatorDeltaPositionCommand(Distance deltaHeight) {
		super(Elevator.getInstance());
		this.deltaHeight = deltaHeight;
	}
	
	@Override
	protected void onStart() {
		this.initialPos = Elevator.getInstance().getPosition();
		Elevator.getInstance().setPosition(initialPos.add(deltaHeight));
	}
	
	@Override
	protected boolean isFinishedNR() {
		boolean finished = Elevator.getInstance().getVelocity().lessThan(Elevator.PROFILE_STOP_SPEED_THRESHOLD)
				&& (initialPos.add(deltaHeight).sub(Elevator.getInstance().getPosition())).abs()
						.lessThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR);
		return finished;
	}
	
}
