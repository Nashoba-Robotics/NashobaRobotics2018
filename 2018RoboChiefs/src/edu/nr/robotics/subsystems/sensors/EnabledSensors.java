package edu.nr.robotics.subsystems.sensors;

import edu.nr.robotics.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;

public class EnabledSensors {

	public static DigitalInput intakeSensor = new DigitalInput(RobotMap.INTAKE_SENSOR_PORT);
	public static DigitalInput elevatorSensor = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT);
	public static DigitalInput portalSensorLeft = new DigitalInput(RobotMap.PORTAL_SENSOR_LEFT_PORT);
	public static DigitalInput portalSensorRight = new DigitalInput(RobotMap.PORTAL_SENSOR_RIGHT_PORT);
	public static DigitalInput cubeHandlerSensor = new DigitalInput(RobotMap.CUBE_HANDLER_SENSOR_PORT);
	public static DigitalInput floorSensor = new DigitalInput(RobotMap.FLOOR_SENSOR_PORT);
	
	public static boolean limelightEnabled = false;
	public static boolean portalSensorEnabled = false;
	public static volatile boolean floorSensorEnabled = false;
	
	public static volatile boolean floorTapeSeen = false;
	
}
