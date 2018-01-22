package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandler;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.sensors.EnableElevatorSensorCommand;

public class CubeFeedIntakeToElevatorCommand extends NRCommand {

	public CubeFeedIntakeToElevatorCommand() {
		super(new NRSubsystem[] {IntakeRollers.getInstance(), CubeHandler.getInstance()});
	}
	
	@Override
	protected void onStart() {
		new EnableElevatorSensorCommand(true).start();
		IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_INTAKE_ROLLERS);
		CubeHandler.getInstance().setMotorSpeedPercent(CubeHandler.VEL_PERCENT_CUBE_HANDLER);
	}
	
	@Override
	protected void onEnd() {
		new EnableElevatorSensorCommand(false).start();
		IntakeRollers.getInstance().disable();
		CubeHandler.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		boolean finished = false;
		if ((IntakeElevator.getInstance().getPosition().sub(IntakeElevator.HANDLER_HEIGHT)).abs().greaterThan(IntakeElevator.PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR)) {
			finished = true;
		}
		//TODO: Get elevator sensor activated to finish CubeFeedIntakeToElevatorCommand
		return finished;
	}
	
	
}
