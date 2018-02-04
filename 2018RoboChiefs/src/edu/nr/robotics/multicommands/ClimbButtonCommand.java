package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.climber.Climber;
import edu.nr.robotics.subsystems.climber.ClimberHoldPositionCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;

public class ClimbButtonCommand extends NRCommand {

	private boolean isClimberTaut;
	
	public static final Distance CLIMB_HEIGHT = Distance.ZERO; //TODO: Find climb height
	
	public ClimbButtonCommand() {
		super(new NRSubsystem[] {Climber.getInstance(), Elevator.getInstance()});
	}

	@Override
	protected void onStart() {
		isClimberTaut = false;
		Climber.getInstance().setCurrent(Climber.DEFAULT_CLIMBER_CURRENT);
	}

	@Override
	protected void onExecute() {
		if (isClimberTaut) {
			Elevator.getInstance().setMotorSpeed(Climber.getInstance().getVelocity().mul(Elevator.HOOK_TO_CARRIAGE_RATIO));
		} else if (Climber.getInstance().getCurrent() >= Climber.MIN_ELEV_CURRENT) {
			isClimberTaut = true;
		}
	}

	@Override
	protected void onEnd() {
		new ClimberHoldPositionCommand().start();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return Elevator.getInstance().getPosition().lessThan(Elevator.CLIMB_HEIGHT_ELEVATOR.sub(CLIMB_HEIGHT));
	}
}
