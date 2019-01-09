package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class ElevatorBottomCommand extends NRCommand {

	public ElevatorBottomCommand() {
		super(Elevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		Elevator.getInstance().setMotorPercentRaw(Elevator.DROP_PERCENT_ELEVATOR);
	}
	
	@Override
	protected void onEnd() {
		Elevator.getInstance().setMotorPercentRaw(0);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return Elevator.getInstance().getPosition().lessThan(new Distance(4, Distance.Unit.INCH));
	}
}
