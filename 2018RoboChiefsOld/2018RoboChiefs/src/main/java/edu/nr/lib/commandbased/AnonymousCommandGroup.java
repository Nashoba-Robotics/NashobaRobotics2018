package edu.nr.lib.commandbased;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * 
 * A CommandGroup that can be created from within another class. This doesn't need a name, so a
 * new class is not needed for every small CommandGroup
 * 
 */

public abstract class AnonymousCommandGroup extends CommandGroup {
	
	public AnonymousCommandGroup() {
		commands();
	}
	
	public abstract void commands();
	
}
