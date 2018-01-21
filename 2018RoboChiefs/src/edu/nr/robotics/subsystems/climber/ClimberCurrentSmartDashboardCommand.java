package edu.nr.robotics.subsystems.climber;

import edu.nr.lib.commandbased.NRCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberCurrentSmartDashboardCommand extends NRCommand {

	
	public ClimberCurrentSmartDashboardCommand() {
		super(Climber.getInstance());
	}
	
	@Override
	protected void onStart() {
		Climber.getInstance().setCurrent(Climber.DEFAULT_CLIMBER_CURRENT);
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}
	
}
