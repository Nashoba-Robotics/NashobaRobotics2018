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

	public static final Distance BASELINE_TO_SWITCH_X = new Distance(168, Distance.Unit.INCH).sub(ROBOT_LENGTH.mul(0.5));
	public static final Distance BASELINE_TO_SWITCH_Y = Distance.ZERO;

	public static final Distance SWITCH_TO_PIVOT_POINT_X = new Distance(88, Distance.Unit.INCH);

	public static final Distance PIVOT_POINT_TO_SCALE_DIAGONAL = new Distance(18, Distance.Unit.INCH);

	public static final Distance PIVOT_POINT_FIELD_EDGE_Y = new Distance(58, Distance.Unit.INCH);

	public static final Distance SWITCH_EDGE_TO_FIELD_EDGE_Y = new Distance(85.25, Distance.Unit.INCH);

	public static final Distance PLATFORM_ZONE_WIDTH_BLOCK = FIELD_WIDTH.sub(new Distance(29.69, Distance.Unit.INCH).mul(2))
			.sub(ROBOT_WIDTH).sub(new Distance(30, Distance.Unit.INCH)); //TODO: Find more preciselty
	
	public static final Distance PLATFORM_ZONE_WIDTH_START_POS = FIELD_WIDTH.sub(BEN_FOOT_LENGTH.mul(2)).sub(ROBOT_WIDTH);

	public static final Distance PIVOT_POINT_TO_CUBE_2_Y = new Distance(113, Distance.Unit.INCH).sub(PIVOT_POINT_FIELD_EDGE_Y);
	public static final Distance PIVOT_POINT_TO_CUBE_3_Y = new Distance(142, Distance.Unit.INCH).sub(PIVOT_POINT_FIELD_EDGE_Y);
	
	public static final Distance CUBE_1_TO_PIVOT_POINT_DIAGONAL = NRMath.hypot(
			SWITCH_EDGE_TO_FIELD_EDGE_Y.sub(PIVOT_POINT_FIELD_EDGE_Y).add(BLOCK_WIDTH.mul(0.5)),
			new Distance(45, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5)));
	
	public static final Distance CUBE_2_TO_PIVOT_POINT_DIAGONAL = NRMath.hypot(
			PIVOT_POINT_TO_CUBE_2_Y.add(BLOCK_WIDTH.mul(0.5)),
			new Distance(45, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5)));
	
	public static final Distance CUBE_3_TO_PIVOT_POINT_DIAGONAL = NRMath.hypot(
			PIVOT_POINT_TO_CUBE_3_Y.add(BLOCK_WIDTH.mul(0.5)),
			new Distance(45, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5)));

	//Distances For Middle Start Position
	public static final Distance PUSH_OFF_WALL_X = new Distance(6, Distance.Unit.INCH);
	public static final Distance START_POS_MID_TO_SWITCH_LEFT_DIAGONAL = NRMath
			.hypot(BASELINE_TO_SWITCH_X.sub(PUSH_OFF_WALL_X), FIELD_WIDTH.mul(0.5).add(new Distance(6,Distance.Unit.INCH)).sub(PIVOT_POINT_FIELD_EDGE_Y)
					.add(ROBOT_WIDTH.mul(0.5).sub(new Distance(12, Distance.Unit.INCH))));
	public static final Distance START_POS_MID_TO_SWITCH_RIGHT_DIAGONAL = NRMath
			.hypot(BASELINE_TO_SWITCH_X.sub(PUSH_OFF_WALL_X), FIELD_WIDTH.mul(0.5).sub(PIVOT_POINT_FIELD_EDGE_Y)
					.sub(ROBOT_WIDTH.mul(0.5).sub(new Distance(12, Distance.Unit.INCH))));

	//Angles For Middle Start Position
	public static final Angle START_POS_MID_TO_SWITCH_ANGLE_LEFT = new Angle(Math.atan((FIELD_WIDTH.mul(0.5).add(new Distance(6, Distance.Unit.INCH))
			.sub(PIVOT_POINT_FIELD_EDGE_Y).add(ROBOT_WIDTH.mul(0.5).sub(new Distance(12, Distance.Unit.INCH))))
					.div(BASELINE_TO_SWITCH_X.sub(PUSH_OFF_WALL_X))),
			Angle.Unit.RADIAN);
	public static final Angle START_POS_MID_TO_SWITCH_ANGLE_RIGHT = new Angle(Math.atan((FIELD_WIDTH.mul(0.5)
			.sub(PIVOT_POINT_FIELD_EDGE_Y).sub(ROBOT_WIDTH.mul(0.5).sub(new Distance(12, Distance.Unit.INCH))))
					.div(BASELINE_TO_SWITCH_X.sub(PUSH_OFF_WALL_X))),
			Angle.Unit.RADIAN);

	public static final Distance CUBE_TO_PLATFORM_ZONE_DIAGONAL = new Distance(40, Distance.Unit.INCH).sub(BLOCK_WIDTH.mul(0.5));
	
	public static final Distance PLATFORM_ZONE_TO_SCALE_DIAGONAL = new Distance(54, Distance.Unit.INCH);
	
	// Platform zone Y direction to diagonal of cube
	public static final Angle PIVOT_POINT_TO_CUBE_1 = new Angle(
			Math.atan((new Distance(45, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5)))
					.div((SWITCH_EDGE_TO_FIELD_EDGE_Y.sub(PIVOT_POINT_FIELD_EDGE_Y).add(BLOCK_WIDTH.mul(0.5))))),
			Angle.Unit.RADIAN);
	public static final Angle PIVOT_POINT_TO_CUBE_2 = new Angle(
			Math.atan((new Distance(45, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5)))
					.div((PIVOT_POINT_TO_CUBE_2_Y.add(BLOCK_WIDTH.mul(0.5))))),
			Angle.Unit.RADIAN);
	public static final Angle PIVOT_POINT_TO_CUBE_3 = new Angle(
			Math.atan((new Distance(45, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5))
					.div((PIVOT_POINT_TO_CUBE_3_Y.add(BLOCK_WIDTH.mul(0.5)))))),
			Angle.Unit.RADIAN);

	// Platform Zone Y direction to diagonal of scale
	public static final Angle PIVOT_POINT_TO_SCALE = new Angle(62, Angle.Unit.DEGREE);
	
	//For switch strafing
	public static final Distance CUBE_1_AND_2_TO_SWITCH_Y = ROBOT_WIDTH.mul(0.5);
	public static final Distance CUBE_3_TO_SWITCH_Y = new Distance(28, Distance.Unit.INCH).add(ROBOT_WIDTH.mul(0.5));
	public static final Distance CUBE_TO_PLATFORM_ZONE_X = new Distance(27, Distance.Unit.INCH).sub(ROBOT_LENGTH.mul(0.5));
	
}
