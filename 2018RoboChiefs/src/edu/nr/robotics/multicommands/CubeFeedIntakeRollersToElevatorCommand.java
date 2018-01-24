package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandler;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class CubeFeedIntakeRollersToElevatorCommand extends NRCommand {

	private boolean finished;
	
	/**
	 * If the IntakeElevator and Elevator are at the right height, the cube will be pushed to the elevator until the elevator
	 * sensor detects that object.
	 */
	public CubeFeedIntakeRollersToElevatorCommand() {
		super(new NRSubsystem[] {IntakeRollers.getInstance()});
	}
	
	@Override
	protected void onStart() {
		
		if ((IntakeElevator.getInstance().getPosition().sub(IntakeElevator.HANDLER_HEIGHT)).abs().greaterThan(IntakeElevator.PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR)) {
			finished = true;
		} else if ((Elevator.getInstance().getPosition().sub(Elevator.BOTTOM_HEIGHT_ELEVATOR)).abs().greaterThan(Elevator.PROFILE_DELTA_POS_THRESHOLD_ELEVATOR)) {
			finished = true;
		} else if (EnabledSensors.intakeSensor.get()) {
			finished = true;
		} else {
			IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_INTAKE_ROLLERS);
		}
	}
	
	@Override
	protected void onEnd() {
		IntakeRollers.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		if (!EnabledSensors.elevatorSensor.get()) {
			finished = true;
		} else {
			finished = false;
		}
		
		return finished;
	}
	
	
}
