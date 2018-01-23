package edu.nr.robotics.subsystems.sensors;

import edu.nr.robotics.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;

public class EnabledSensors {

	public static DigitalInput intakeSensor = new DigitalInput(RobotMap.INTAKE_SENSOR_PORT);
	public static DigitalInput elevatorSensor = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT);
	public static DigitalInput portalSensorLeft = new DigitalInput(RobotMap.PORTAL_SENSOR_LEFT_PORT);
	public static DigitalInput portalSensorRight = new DigitalInput(RobotMap.PORTAL_SENSOR_RIGHT_PORT);
	public static DigitalInput outletSensor = new DigitalInput(RobotMap.OUTLET_SENSOR_PORT);
	
	public static boolean intakeSensorEnabled = false;
	public static boolean elevatorSensorEnabled = false;
	public static boolean limelightEnabled = false;
	public static boolean portalSensorEnabled = false;
	public static boolean outletSensorEnabled = false; //TODO: See what other sensors we need
	//public static boolean floorSensorEnabled = false;
	
}
