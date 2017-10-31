package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.driving.DriveTypeCalculations;

public class CheesyDriveCalculationConstants {

	public static final double HIGH_TURN_NON_LINEARITY = 0.65;
	public static final double LOW_TURN_NON_LINEARITY = 0.5;
	
	public static final double HIGH_NEG_INERTIA_SCALAR = 4.0;
	
	public static final double LOW_NEG_INERTIA_SCALAR = 0.65;
	public static final double LOW_NEG_INERTIA_TURN_SCALAR = 3.5;
	public static final double LOW_NEG_INERTIA_CLOSE_SCALAR = 4.0;
	public static final double LOW_NEG_INERTIA_FAR_SCALAR = 5.0;
	
	public static final double HIGH_SENSITIVITY = 0.95;
	public static final double LOW_SENSITIVITY = 1.3;
	
	public static final double QUICK_STOP_DEADBAND = 0.2;
	public static final double QUICK_STOP_WEIGHT = 0.1;
	public static final double QUICK_STOP_SCALAR = 5.0;
	
	public static void createDriveTypeCalculations() {
		new DriveTypeCalculations(HIGH_TURN_NON_LINEARITY, LOW_TURN_NON_LINEARITY, HIGH_NEG_INERTIA_SCALAR, LOW_NEG_INERTIA_SCALAR, 
				LOW_NEG_INERTIA_TURN_SCALAR, LOW_NEG_INERTIA_CLOSE_SCALAR, LOW_NEG_INERTIA_FAR_SCALAR, HIGH_SENSITIVITY, LOW_SENSITIVITY, 
				QUICK_STOP_DEADBAND, QUICK_STOP_WEIGHT, QUICK_STOP_SCALAR);
	}
}
