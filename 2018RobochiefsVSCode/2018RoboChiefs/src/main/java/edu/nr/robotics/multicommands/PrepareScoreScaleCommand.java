package edu.nr.robotics.multicommands;

import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

public class PrepareScoreScaleCommand extends CommandGroup {

	public PrepareScoreScaleCommand() {
		
		addSequential(new ConditionalCommand(new ElevatorProfileCommandGroup(Elevator.SCALE_HEIGHT_ELEVATOR, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR)) {
			
			@Override
			protected boolean condition() {
				//return !EnabledSensors.elevatorSensor.get();
				return true;
			}
		});
		
	}
}