package edu.nr.lib.commandbased;

/**
 * A {@link JoystickCommand} that doesn't do anything.
 * Used for subsystems that need a default command but do not have an associated joystick.
 *
 */
public final class DoNothingJoystickCommand extends JoystickCommand {
	
	/**
	 * Create the command
	 * @param subsystem The subsystem to associate the command with
	 */
	public DoNothingJoystickCommand(NRSubsystem s) {
		super(s);
	}

	/**
	 * Condition for switching to reading values from the joystick.
	 */
	@Override
	protected boolean shouldSwitchToJoystick() {
		return false;
	}

	/**
	 * How long we should wait between checks to switch to the joystick.
	 */
	@Override
	protected long getPeriodOfCheckingForSwitchToJoystick() {
		return Long.MAX_VALUE;
	}
	
}
