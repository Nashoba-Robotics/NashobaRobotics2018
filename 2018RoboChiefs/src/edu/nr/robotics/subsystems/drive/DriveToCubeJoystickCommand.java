package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.OI;

public class DriveToCubeJoystickCommand extends NRCommand {

	public DriveToCubeJoystickCommand() {
		super(Drive.getInstance());
	}
	
	@Override
	protected void onStart() {
		LimelightNetworkTable.getInstance().enable();
	}
	
	@Override
	protected void onExecute() {
		
		double moveValue = OI.getInstance().getArcadeMoveValue();
				
		double headingAdjustment;
		
		headingAdjustment = ((-Math.cos(LimelightNetworkTable.getInstance().getHorizOffset().get(Angle.Unit.RADIAN) 
				/ ((Drive.DRIVE_STOP_ANGLE.get(Angle.Unit.DEGREE) / 90) * 3)) 
				* (1 - Drive.MIN_PROFILE_TURN_PERCENT)) + 1 + Drive.MIN_PROFILE_TURN_PERCENT) 
				* -LimelightNetworkTable.getInstance().getHorizOffset().signum();
		if (Math.abs(headingAdjustment) < Drive.MIN_PROFILE_TURN_PERCENT) {
			headingAdjustment = Drive.MIN_PROFILE_TURN_PERCENT * Math.signum(headingAdjustment);
		}
		
		double outputLeft = moveValue - headingAdjustment;
		double outputRight = moveValue + headingAdjustment;
		
		Drive.getInstance().setMotorSpeedInPercent(outputLeft, outputRight, 0);
	}
	
	@Override
	protected void onEnd() {
		LimelightNetworkTable.getInstance().disable();
	}
	
	@Override
	protected boolean isFinishedNR() {
		return false;
	}
}
