package edu.nr.robotics.subsystems.elevator;

import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.units.Acceleration;
import edu.nr.lib.units.Speed;

public class Elevator extends NRSubsystem{

	private static Elevator elevator;
	
	/**
	 * The max speed of the elevator
	 */
	public static final Speed MAX_SPEED = Speed.ZERO; //TODO: Find real elevator max speed
	
	/**
	 * The max acceleration of the elevator
	 */
	public static final Acceleration MAX_ACCELERATION = Acceleration.ZERO; //TODO: Find real elevator max acceleration
	
	/**
	 * The elevator voltage-velocity curve slopes
	 */
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_LEFT = 0; //TODO: Find elevator voltage vs velocity curve
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_RIGHT = 0;
	public static final double VOLTAGE_PERCENT_VELOCITY_SLOPE_H = 0;
	
	/**
	 * Voltage percentage at which elevator just starts moving
	 */
	public static final double MIN_MOVE_VOLTAGE_PERCENT_LEFT = 0; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_RIGHT = 0; //This is 0 to 1 number
	public static final double MIN_MOVE_VOLTAGE_PERCENT_H = 0; //This si 0 to 1 number
	
	@Override
	public void smartDashboardInfo() {
		
	}

	@Override
	public void disable() {
		
	}

}
