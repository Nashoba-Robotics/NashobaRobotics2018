package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class IntakeElevatorBottomCommand extends NRCommand {

	public IntakeElevatorBottomCommand() {
		super(IntakeElevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		IntakeElevator.getInstance().setMotorSpeedPercent(-0.3);
		IntakeElevator.intakeFolded = false;
	}
	
	@Override
	protected boolean isFinishedNR() {
		return IntakeElevator.getInstance().getPosition().equals(Distance.ZERO);
	}
}
