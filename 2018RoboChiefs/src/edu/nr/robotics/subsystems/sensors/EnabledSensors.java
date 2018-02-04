package edu.nr.robotics.subsystems.sensors;

import edu.nr.robotics.RobotMap;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

public class EnabledSensors {

	public static DigitalInput intakeSensor = new DigitalInput(RobotMap.INTAKE_SENSOR_PORT);
	public static DigitalInput elevatorSensor = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT);
	public static DigitalInput portalSensorLeft1 = new DigitalInput(RobotMap.PORTAL_SENSOR_LEFT_1_PORT);
	public static DigitalInput portalSensorLeft2 = new DigitalInput(RobotMap.PORTAL_SENSOR_LEFT_2_PORT);
	public static DigitalInput portalSensorRight1 = new DigitalInput(RobotMap.PORTAL_SENSOR_RIGHT_1_PORT);
	public static DigitalInput portalSensorRight2 = new DigitalInput(RobotMap.PORTAL_SENSOR_RIGHT_2_PORT);
	public static DigitalInput cubeHandlerSensor = new DigitalInput(RobotMap.CUBE_HANDLER_SENSOR_PORT);
	public static DigitalInput floorSensor = new DigitalInput(RobotMap.FLOOR_SENSOR_PORT);
	
	public static boolean limelightEnabled = false;
	public static boolean portalSensorEnabled = false;
	public static volatile boolean floorSensorEnabled = false;
	
	public static Counter floorCounter = new Counter(floorSensor);
	
	public static boolean portalReached = false;
	
}
