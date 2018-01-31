package edu.nr.robotics.auton;

import edu.nr.lib.NRMath;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;

public class FieldMeasurements {

	public static final Distance ROBOT_LENGTH = new Distance(0, Distance.Unit.INCH); // TODO:
																						// Find
																						// length
																						// of
																						// robot
	public static final Distance ROBOT_WIDTH = new Distance(0, Distance.Unit.INCH); // TODO:
																					// Find
																					// width
																					// of
																					// robot

	public static final Distance BLOCK_WIDTH = new Distance(13, Distance.Unit.INCH);
	
	public static final Distance BEN_FOOT_LENGTH = new Distance(13, Distance.Unit.INCH);

	public static final Distance FIELD_WIDTH = new Distance(324, Distance.Unit.INCH);

	public static final Distance BASELINE_TO_PLATFORM_ZONE_X = new Distance(236, Distance.Unit.INCH)
			.sub(ROBOT_LENGTH.mul(0.5));
	public static final Distance BASELINE_TO_PLATFORM_ZONE_Y = Distance.ZERO;
	
	public static final Distance BASELINE_TO_SCALE_X = new Distance(254, Distance.Unit.INCH);

	public static final Distance BASELINE_TO_SWITCH_X = new Distance(168, Distance.Unit.INCH).sub(ROBOT_LENGTH.mul(2));
	public static final Distance BASELINE_TO_SWITCH_Y = Distance.ZERO;

	public static final Distance SWITCH_TO_PLATFORM_ZONE_X = BASELINE_TO_PLATFORM_ZONE_X.sub(BASELINE_TO_SWITCH_X);
	public static final Distance SWITCH_TO_PLATFORM_ZONE_Y = Distance.ZERO;

	public static final Distance PLATFORM_ZONE_TO_SCALE_DIAGONAL = new Distance(54, Distance.Unit.INCH).mul(0.5);

	public static final Distance FAR_START_POS_TO_FIELD_EDGE_Y = new Distance(29.7, Distance.Unit.INCH)
			.add(ROBOT_WIDTH.mul(0.5));

	public static final Distance SWITCH_EDGE_TO_FIELD_EDGE_Y = new Distance(85.25, Distance.Unit.INCH);

	public static final Distance FAR_START_POS_TO_OUTER_BLOCK_Y = SWITCH_EDGE_TO_FIELD_EDGE_Y
			.sub(FAR_START_POS_TO_FIELD_EDGE_Y).sub(ROBOT_LENGTH.mul(0.5)).add(BLOCK_WIDTH.mul(0.5));

	public static final Distance PLATFORM_ZONE_WIDTH = FIELD_WIDTH.sub(FAR_START_POS_TO_FIELD_EDGE_Y.mul(2));
	
	public static final Distance PLATFORM_ZONE_WIDTH_SHORT = PLATFORM_ZONE_WIDTH.sub(BEN_FOOT_LENGTH);

	public static final Distance CUBE_TO_PLATFORM_ZONE_X = new Distance(0, Distance.Unit.INCH); // TODO:
																								// Find
																								// this

	public static final Distance CUBE_TO_PLATFORM_ZONE_DIAGONAL = NRMath.hypot(
			FieldMeasurements.BASELINE_TO_PLATFORM_ZONE_X.sub(new Distance(196, Distance.Unit.INCH)).add(ROBOT_LENGTH.mul(0.5))
					.sub(FieldMeasurements.BLOCK_WIDTH.mul(0.5)),
			FieldMeasurements.SWITCH_EDGE_TO_FIELD_EDGE_Y.sub(FieldMeasurements.FAR_START_POS_TO_FIELD_EDGE_Y)
					.add(FieldMeasurements.BLOCK_WIDTH.mul(0.5)));
	
	//Distances For Middle Start Position
	public static final Distance PUSH_OFF_WALL_X = new Distance(6, Distance.Unit.INCH);
	public static final Distance START_POS_MID_TO_SWITCH_LEFT_DIAGONAL = NRMath
			.hypot(BASELINE_TO_SWITCH_X.sub(PUSH_OFF_WALL_X), FIELD_WIDTH.mul(0.5).add(new Distance(6,Distance.Unit.INCH)).sub(FAR_START_POS_TO_FIELD_EDGE_Y)
					.add(ROBOT_WIDTH.mul(0.5).sub(new Distance(12, Distance.Unit.INCH))));
	public static final Distance START_POS_MID_TO_SWITCH_RIGHT_DIAGONAL = NRMath
			.hypot(BASELINE_TO_SWITCH_X.sub(PUSH_OFF_WALL_X), FIELD_WIDTH.mul(0.5).sub(FAR_START_POS_TO_FIELD_EDGE_Y)
					.sub(ROBOT_WIDTH.mul(0.5).sub(new Distance(12, Distance.Unit.INCH))));

	//Angles For Middle Start Position
	public static final Angle START_POS_MID_TO_SWITCH_ANGLE_LEFT = new Angle(Math.atan((FIELD_WIDTH.mul(0.5).add(new Distance(6, Distance.Unit.INCH))
			.sub(FAR_START_POS_TO_FIELD_EDGE_Y).add(ROBOT_WIDTH.mul(0.5).sub(new Distance(12, Distance.Unit.INCH))))
					.div(BASELINE_TO_SWITCH_X.sub(PUSH_OFF_WALL_X))),
			Angle.Unit.RADIAN);
	public static final Angle START_POS_MID_TO_SWITCH_ANGLE_RIGHT = new Angle(Math.atan((FIELD_WIDTH.mul(0.5)
			.sub(FAR_START_POS_TO_FIELD_EDGE_Y).sub(ROBOT_WIDTH.mul(0.5).sub(new Distance(12, Distance.Unit.INCH))))
					.div(BASELINE_TO_SWITCH_X.sub(PUSH_OFF_WALL_X))),
			Angle.Unit.RADIAN);

	
	// Platform zone Y direction to diagonal of cube
	public static final Angle PLATFORM_ZONE_TO_CUBE = new Angle(
			Math.atan(((BASELINE_TO_PLATFORM_ZONE_X.sub(new Distance(196, Distance.Unit.INCH)).sub(BLOCK_WIDTH))
					.div(SWITCH_EDGE_TO_FIELD_EDGE_Y.sub(FAR_START_POS_TO_FIELD_EDGE_Y)))),
			Angle.Unit.RADIAN);

	// Platform Zone Y direction to diagonal of scale
	public static final Angle PLATFORM_ZONE_TO_SCALE = new Angle(62, Angle.Unit.DEGREE);
}
