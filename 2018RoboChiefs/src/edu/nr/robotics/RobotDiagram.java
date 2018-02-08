package edu.nr.robotics;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class RobotDiagram implements Sendable {
//TODO: Fix Errors
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

	private NetworkTable table;
	private NetworkTableInstance tableInstance;

	private RobotDiagram() {
		tableInstance = NetworkTableInstance.create();
		table = tableInstance.getTable(getName());
	}

	/* This is what was here before the new wpilib
	@Override
	public void initTable(NetworkTable subtable) {
		this.table = subtable;
		if (table != null) {
			table.add("Match Time", DriverStation.getInstance().getMatchTime());
			table.putBoolean("Is Auto", Robot.getInstance().isAutonomous());
			
			table.putNumber("Current robot time", Timer.getFPGATimestamp());

		}

	}
	

	@Override
	public NetworkTable getTable() {
		return table;
	}

	@Override
	public String getSmartDashboardType() {
		return "Robot Diagram";
	}
	*/

	@Override
	public String getName() {
		return "Robot Diagram";
	}

	@Override
	public void setName(String name) {
		
	}

	@Override
	public String getSubsystem() {
		return null;
	}

	@Override
	public void setSubsystem(String subsystem) {
		
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		//this.tableInstance = builder; dunno
		if (builder != null) {
			builder.getEntry("Match Time");
			builder.getEntry("Match Time").setDouble(DriverStation.getInstance().getMatchTime());
			builder.getEntry("Is Auto").setBoolean(Robot.getInstance().isAutonomous());
			
			builder.getEntry("Current robot time").setDouble(Timer.getFPGATimestamp());

		}
		
	}

}
