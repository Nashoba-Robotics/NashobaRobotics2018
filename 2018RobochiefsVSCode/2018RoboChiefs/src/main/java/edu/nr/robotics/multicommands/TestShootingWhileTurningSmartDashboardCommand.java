package edu.nr.robotics.multicommands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.nr.lib.commandbased.AnonymousCommandGroup;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Time;
import edu.nr.robotics.auton.FieldMeasurements;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterShootCommand;

public class TestShootingWhileTurningSmartDashboardCommand extends CommandGroup {

	public TestShootingWhileTurningSmartDashboardCommand() {

		/*addParallel(new TurnCommand(Drive.getInstance(),
				new Angle(90, Angle.Unit.DEGREE).add(FieldMeasurements.PIVOT_POINT_TO_CUBE_1),
				Drive.MAX_PROFILE_TURN_PERCENT));*/
		
		addParallel(new TurnCommand(Drive.getInstance(),(new Angle(360, Angle.Unit.DEGREE).add(FieldMeasurements.CUBE_1_TO_CUBE_2)).negate(), Drive.MAX_PROFILE_TURN_PERCENT));

		addParallel(new AnonymousCommandGroup() {

			@Override
			public void commands() {

				/*addSequential(
						new WaitCommand(FieldMeasurements.AUTO_SHOOT_TURNING_FIRST_WAIT_TIME.get(Time.Unit.SECOND)));*/
				addSequential(
						new WaitCommand(FieldMeasurements.AUTO_SHOOT_TURNING_SECOND_WAIT_TIME.get(Time.Unit.SECOND)));
				addSequential(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_SCALE_AUTO_ELEVATOR_SHOOTER));

			}
		});
	}
}
