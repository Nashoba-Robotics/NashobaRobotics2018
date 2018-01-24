package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandler;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;

public class CubeFeedIntakeRollersToCubeHandlerCommand extends NRCommand {

	/**
	 * If the IntakeElevator and Elevator are at the right height, the cube will be pushed to the back until the cube handler
	 * sensor detects that cube.
	 */
	public CubeFeedIntakeRollersToCubeHandlerCommand() {
		super(new NRSubsystem[] {IntakeRollers.getInstance(), CubeHandler.getInstance()});
	}
	
	@Override
	protected void onStart() {
		IntakeRollers.getInstance().setMotorSpeedPercent(IntakeRollers.VEL_PERCENT_INTAKE_ROLLERS);
		CubeHandler.getInstance().setMotorSpeedPercent(CubeHandler.VEL_PERCENT_CUBE_HANDLER);
	}
	
	@Override
	protected void onEnd() {
		IntakeRollers.getInstance().disable();
		CubeHandler.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		if ((IntakeElevator.getInstance().getPosition().sub(IntakeElevator.HANDLER_HEIGHT)).abs().greaterThan(IntakeElevator.PROFILE_DELTA_POS_THRESHOLD_INTAKE_ELEVATOR)) {
			return true;
		}
		if (Elevator.getInstance().getPosition().sub(Elevator.SWITCH_HEIGHT_ELEVATOR.sub(Elevator.PROFILE_DELTA_POS_THRESHOLD_ELEVATOR)).greaterThan(Elevator.PROFILE_DELTA_POS_THRESHOLD_ELEVATOR)) {
			return true;
		}
		
		return !EnabledSensors.cubeHandlerSensor.get();
	}
	
}
