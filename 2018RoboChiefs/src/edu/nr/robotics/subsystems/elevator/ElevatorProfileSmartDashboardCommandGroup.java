package edu.nr.robotics.subsystems.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ElevatorProfileSmartDashboardCommandGroup extends CommandGroup {

	public ElevatorProfileSmartDashboardCommandGroup() {
		
		addSequential(new ElevatorProfileBasicSmartDashboardCommand());
		//addSequential(new ElevatorPositionSmartDashboardCommand());
	}
}
