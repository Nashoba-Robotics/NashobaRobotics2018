package edu.nr.robotics.subsystems.sensors;

import edu.nr.robotics.RobotMap;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

public class EnabledSensors {

	public static DigitalInput intakeSensorLeft = new DigitalInput(RobotMap.INTAKE_SENSOR_PORT_LEFT);
	public static DigitalInput intakeSensorRight = new DigitalInput(RobotMap.INTAKE_SENSOR_PORT_RIGHT);
	public static DigitalInput elevatorSensor = new DigitalInput(RobotMap.ELEVATOR_SENSOR_PORT);
	public static DigitalInput floorSensor = new DigitalInput(RobotMap.FLOOR_SENSOR_PORT);

	public static DigitalInput forceSensor1 = new DigitalInput(RobotMap.FORCE_SENSOR_ONE_PORT);
	public static DigitalInput forceSensor2 = new DigitalInput(RobotMap.FORCE_SENSOR_TWO_PORT);
	
	public static boolean limelightEnabled = false;
	public static boolean portalSensorEnabled = false;
	public static volatile boolean floorSensorEnabled = false;
	
	public static Counter floorCounter = new Counter(floorSensor);
		
}
