package edu.nr.robotics.subsystems.intakeElevator;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class IntakeElevatorProfileSmartDashboardCommandGroup extends CommandGroup {

	public IntakeElevatorProfileSmartDashboardCommandGroup() {
		
		addSequential(new IntakeElevatorProfileBasicSmartDashboardCommand());
		//addSequential(new IntakeElevatorPositionSmartDashboardCommand());
	}
}
