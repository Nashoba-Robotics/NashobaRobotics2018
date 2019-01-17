package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.elevator.Elevator;

public class IntakeElevatorDeltaPositionCommand extends NRCommand {
	
	private Distance initialPos;
	private Distance deltaHeight;
	
	/**
	 * Sets the intake elevator position to the change specified
	 * @param deltaHeight: The change in height to go. Up is positive and down is negative
	 */
	public IntakeElevatorDeltaPositionCommand(Distance deltaHeight) {
		super(IntakeElevator.getInstance());
		this.deltaHeight = deltaHeight;
	}
	
	@Override
	protected void onStart() {
		this.initialPos = IntakeElevator.getInstance().getPosition();
		IntakeElevator.getInstance().setPosition(initialPos.add(deltaHeight));
	}
	
	@Override
	protected boolean isFinishedNR() {
		boolean finished = IntakeElevator.getInstance().getVelocity().lessThan(IntakeElevator.PROFILE_STOP_SPEED_THRESHOLD)
				&& (initialPos.add(deltaHeight).sub(IntakeElevator.getInstance().getPosition())).abs()
						.lessThan(IntakeElevator.PROFILE_END_POS_THRESHOLD_INTAKE_ELEVATOR);
		return finished;
	}

}
