package edu.nr.robotics.auton;

import edu.nr.lib.units.Distance;

public class FieldDistances {

	public static final Distance ROBOT_LENGTH = new Distance(38, Distance.Unit.INCH);
	public static final Distance ROBOT_WIDTH = new Distance(28, Distance.Unit.INCH);
	
	public static final Distance BLOCK_WIDTH = new Distance(13, Distance.Unit.INCH);
	
	public static final Distance FIELD_WIDTH = new Distance(324, Distance.Unit.INCH);
	
	public static final Distance BASELINE_TO_SCALE_X = new Distance(324, Distance.Unit.INCH).sub(ROBOT_LENGTH.mul(0.5));
	public static final Distance BASELINE_TO_SCALE_Y = Distance.ZERO;
	
	public static final Distance BASELINE_TO_PLATFORM_ZONE_X = new Distance(236, Distance.Unit.INCH).sub(ROBOT_LENGTH.mul(0.5));
	public static final Distance BASELINE_TO_PLATFORM_ZONE_Y = Distance.ZERO;
	
	public static final Distance BASELINE_TO_SWITCH_X = new Distance(168, Distance.Unit.INCH).sub(ROBOT_LENGTH.mul(0.5));
	public static final Distance BASELINE_TO_SWITCH_Y = Distance.ZERO;
	
	public static final Distance SWITCH_TO_PLATFORM_ZONE_X = BASELINE_TO_PLATFORM_ZONE_X.sub(BASELINE_TO_SWITCH_X);
	public static final Distance SWITCH_TO_PLATFORM_ZONE_Y = Distance.ZERO;
	
	public static final Distance PLATFORM_ZONE_TO_SCALE_X = BASELINE_TO_SCALE_X.sub(BASELINE_TO_PLATFORM_ZONE_X);
	public static final Distance PLATFORM_ZONE_TO_SCALE_Y = Distance.ZERO;
	
	public static final Distance FAR_START_POS_TO_FIELD_EDGE_Y = new Distance(29.69, Distance.Unit.INCH);
	
	public static final Distance SWITCH_EDGE_TO_FIELD_EDGE_Y = new Distance(85.25, Distance.Unit.INCH);
	
	public static final Distance CUBE_TO_PLATFORM_ZONE_X = new Distance(44, Distance.Unit.INCH).sub(ROBOT_LENGTH.mul(0.5));
	
	public static final Distance FAR_START_POS_TO_OUTER_BLOCK_Y = SWITCH_EDGE_TO_FIELD_EDGE_Y.sub(FAR_START_POS_TO_FIELD_EDGE_Y)
			.sub(ROBOT_LENGTH.mul(0.5)).add(BLOCK_WIDTH.mul(0.5));
	
	public static final Distance PLATFORM_ZONE_WIDTH = FIELD_WIDTH.sub(FAR_START_POS_TO_FIELD_EDGE_Y.mul(2));
}
