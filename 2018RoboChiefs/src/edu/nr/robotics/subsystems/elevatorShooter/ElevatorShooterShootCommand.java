package edu.nr.robotics.subsystems.elevatorShooter;

import edu.nr.lib.units.Time;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ElevatorShooterShootCommand extends CommandGroup {
	
	public ElevatorShooterShootCommand(double percent) {

		addParallel(new ElevatorShooterVelocityCommand(percent));
		addSequential(new WaitCommand(ElevatorShooter.SHOOT_TIME.get(Time.Unit.SECOND)));
		addSequential(new ElevatorShooterStopCommand());
	}
}
