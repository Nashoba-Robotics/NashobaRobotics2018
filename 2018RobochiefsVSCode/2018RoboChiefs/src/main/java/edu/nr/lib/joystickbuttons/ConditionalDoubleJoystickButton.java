package edu.nr.lib.joystickbuttons;

import edu.nr.lib.joystickbuttons.DoubleJoystickButton.Type;
import edu.wpi.first.wpilibj.buttons.Button;

public class ConditionalDoubleJoystickButton extends Button {

	private Button m_buttonNumberOne;
	private Button m_buttonNumberTwo;
	private boolean condition;
	
	Type type;
	
	public ConditionalDoubleJoystickButton(Button buttonNumberOne, Button buttonNumberTwo, Type type, boolean condition) {
		m_buttonNumberOne = buttonNumberOne;
		m_buttonNumberTwo = buttonNumberTwo;
		this.type = type;
		this.condition = condition;
	}
	
	@Override
	public boolean get() {
		if(type == Type.And) {
			return (m_buttonNumberOne.get() && m_buttonNumberTwo.get()) && condition;
		} else {
			return (m_buttonNumberOne.get() || m_buttonNumberTwo.get()) && condition;
		}
	}
	
}
