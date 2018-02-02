package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.subsystems.climber.Climber;
import edu.nr.robotics.subsystems.climber.ClimberHoldPositionCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;

public class ClimbCommand extends NRCommand {

	public static final Distance CLIMB_HEIGHT = Distance.ZERO;//TODO: Find climb height

	public ClimbCommand() {
		super(new NRSubsystem[] {Climber.getInstance(), Elevator.getInstance()});
	}

	@Override
	protected void onStart() {
		Climber.getInstance().setCurrent(Climber.DEFAULT_CLIMBER_CURRENT);
	}

	@Override
	protected void onExecute() {
		Elevator.getInstance().setMotorSpeed(Climber.getInstance().getVelocity());
	}

	@Override
	protected void onEnd() {
		new ClimberHoldPositionCommand().start();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return Elevator.getInstance().getPosition().lessThan(Elevator.TOP_HEIGHT_ELEVATOR.sub(CLIMB_HEIGHT));
	}
}
