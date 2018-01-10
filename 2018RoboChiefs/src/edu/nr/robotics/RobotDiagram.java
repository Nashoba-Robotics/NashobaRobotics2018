package edu.nr.robotics;

import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Angle.Unit;
import edu.nr.lib.units.Time;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.NamedSendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.tables.ITable;

public class RobotDiagram implements NamedSendable {

	private static RobotDiagram singleton;

	public static RobotDiagram getInstance() {
		if (singleton == null) {
			init();
		}
		return singleton;
	}

	public synchronized static void init() {
		if (singleton == null) {
			singleton = new RobotDiagram();
		}
	}

	private ITable table;

	private RobotDiagram() {
	}

	@Override
	public void initTable(ITable subtable) {
		this.table = subtable;
		if (table != null) {
			table.putNumber("Match Time", DriverStation.getInstance().getMatchTime());
			table.putBoolean("Is Auto", Robot.getInstance().isAutonomous());
			
			table.putNumber("Current robot time", Timer.getFPGATimestamp());

		}

	}

	@Override
	public ITable getTable() {
		return table;
	}

	@Override
	public String getSmartDashboardType() {
		return "Robot Diagram";
	}

	@Override
	public String getName() {
		return "Robot Diagram";
	}

}
