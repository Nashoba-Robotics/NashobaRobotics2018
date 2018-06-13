package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.commandbased.NRCommand;

public class EnableSniperTurnMode extends NRCommand {

	private boolean bool;
	
	public static final double TURN_JOYSTICK_MULTIPLIER_LOW = 0.5;
	public static final double TURN_JOYSTICK_MULTIPLIER_HIGH = 1.0;
	
	public EnableSniperTurnMode(boolean bool) {
		this.bool = bool;
	}
	
	@Override
	protected void onStart() {
		if (bool) {
			Drive.TURN_JOYSTICK_MULTIPLIER = TURN_JOYSTICK_MULTIPLIER_LOW;
		} else {
			Drive.TURN_JOYSTICK_MULTIPLIER = TURN_JOYSTICK_MULTIPLIER_HIGH;
		}
		
		Drive.sniperModeEnabled = bool;
	}
}