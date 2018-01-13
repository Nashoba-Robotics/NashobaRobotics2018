package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.driving.DriveTypeCalculations;

public class CheesyDriveCalculationConstants {

	public static final double HIGH_NEG_THRESHOLD = 0;
	public static final double HIGH_NEG_INERTIA_TURN_SCALAR = 0;
	public static final double HIGH_NEG_INERTIA_CLOSE_SCALAR = 0;
	public static final double HIGH_NEG_INERTIA_FAR_SCALAR = 0;
	
	public static final double LOW_NEG_THRESHOLD = 0.5;//0.65
	public static final double LOW_NEG_INERTIA_TURN_SCALAR = 3.0;//3.5
	public static final double LOW_NEG_INERTIA_CLOSE_SCALAR = 80.0;//4.0
	public static final double LOW_NEG_INERTIA_FAR_SCALAR = 100.0;//5.0
	
	public static void createDriveTypeCalculations() {
		new DriveTypeCalculations(HIGH_NEG_THRESHOLD, HIGH_NEG_INERTIA_TURN_SCALAR, HIGH_NEG_INERTIA_CLOSE_SCALAR, HIGH_NEG_INERTIA_FAR_SCALAR, 
								LOW_NEG_THRESHOLD, LOW_NEG_INERTIA_TURN_SCALAR, LOW_NEG_INERTIA_CLOSE_SCALAR, LOW_NEG_INERTIA_FAR_SCALAR);
	}
}
