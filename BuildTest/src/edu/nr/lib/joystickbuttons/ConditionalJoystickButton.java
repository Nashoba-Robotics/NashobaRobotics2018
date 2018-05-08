package edu.nr.lib.joystickbuttons;

import edu.wpi.first.wpilibj.buttons.Button;

public class ConditionalJoystickButton extends Button {

	private boolean condition;
	Button button;
	
	public ConditionalJoystickButton(Button button, boolean condition) {
		this.button = button;
		this.condition = condition;
	}
	
	@Override
	public boolean get() {
		return button.get() && condition;
	}

}
