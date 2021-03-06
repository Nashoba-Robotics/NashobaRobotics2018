package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorPositionCommand extends NRCommand {

	private Distance height;
	
	public ElevatorPositionCommand(Distance height) {
		super(Elevator.getInstance());
		this.height = height;
	}
	
	@Override
	protected void onStart() {
		Elevator.getInstance().setPosition(height);
	}
	
	@Override
	protected boolean isFinishedNR() {
		
		boolean finished = Elevator.getInstance().getVelocity().lessThan(Elevator.PROFILE_STOP_SPEED_THRESHOLD)
		&& (Elevator.getInstance().getPosition().sub(height)).abs().lessThan(Elevator.PROFILE_END_POS_THRESHOLD_ELEVATOR);
		
		return finished;
	}
}
