package edu.nr.robotics.subsystems.drive;

import edu.nr.lib.driving.DriveTypeCalculations;

public class CheesyDriveCalculationConstants {

	public static final double HIGH_NEG_THRESHOLD = 0;
	public static final double HIGH_NEG_INERTIA_TURN_SCALAR = 0;
	public static final double HIGH_NEG_INERTIA_CLOSE_SCALAR = 0;
	public static final double HIGH_NEG_INERTIA_FAR_SCALAR = 0;
	
	public static final double LOW_NEG_THRESHOLD = 0.65;
	public static final double LOW_NEG_INERTIA_TURN_SCALAR = 3.5;
	public static final double LOW_NEG_INERTIA_CLOSE_SCALAR = 4.0;
	public static final double LOW_NEG_INERTIA_FAR_SCALAR = 5.0;
	
	public static void createDriveTypeCalculations() {
		new DriveTypeCalculations(HIGH_NEG_THRESHOLD, HIGH_NEG_INERTIA_TURN_SCALAR, HIGH_NEG_INERTIA_CLOSE_SCALAR, HIGH_NEG_INERTIA_FAR_SCALAR, 
								HIGH_NEG_THRESHOLD, HIGH_NEG_INERTIA_TURN_SCALAR, HIGH_NEG_INERTIA_CLOSE_SCALAR, HIGH_NEG_INERTIA_FAR_SCALAR);
	}
}
