package edu.nr.robotics.multicommands;

import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class PrepareScoreSwitchCommand extends CommandGroup {
	
	public PrepareScoreSwitchCommand() {
		
		addSequential(new ConditionalCommand(new AnonymousCommandGroup() {
			
			@Override
			public void commands() {
				addParallel(new ElevatorProfileCommandGroup(Elevator.SWITCH_HEIGHT_ELEVATOR,
						Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
				
				addParallel(new AnonymousCommandGroup() {
					
					@Override
					public void commands() {
						addSequential(new WaitCommand(0.5));
						addSequential(new FoldIntakeMultiCommand());
					}
				});
				
			}
		}) {
			
			@Override
			protected boolean condition() {
				return !EnabledSensors.elevatorSensor.get();
			}
		});
	
	}
}
