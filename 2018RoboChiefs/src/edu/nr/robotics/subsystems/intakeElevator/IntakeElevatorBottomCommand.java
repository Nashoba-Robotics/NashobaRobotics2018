package edu.nr.robotics.subsystems.intakeElevator;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.units.Distance;

public class IntakeElevatorBottomCommand extends NRCommand {

	public IntakeElevatorBottomCommand() {
		super(IntakeElevator.getInstance());
	}
	
	@Override
	protected void onStart() {
		IntakeElevator.getInstance().setMotorSpeedPercent(-IntakeElevator.PROFILE_VEL_PERCENT_INTAKE_ELEVATOR);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return IntakeElevator.getInstance().getPosition().lessThan(new Distance(1, Distance.Unit.INCH));
	}
}
