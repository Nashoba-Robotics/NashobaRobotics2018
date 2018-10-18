package edu.nr.robotics.auton;

import edu.nr.lib.NRMath;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;

public class FieldMeasurements {

	public static final Time AUTO_SHOOT_TURNING_FIRST_WAIT_TIME = new Time(0, Time.Unit.SECOND);
	
	public static final Time AUTO_SHOOT_TURNING_SECOND_WAIT_TIME = new Time(0.25, Time.Unit.SECOND);
	
	public static final Distance ROBOT_LENGTH = new Distance(33, Distance.Unit.INCH);
	
	public static final Distance ROBOT_WIDTH = new Distance(27, Distance.Unit.INCH); 
	
	public static final Distance INTAKE_LENGTH = new Distance(15, Distance.Unit.INCH);

	public static final Distance BLOCK_WIDTH = new Distance(13, Distance.Unit.INCH);
	
	public static final Distance BEN_FOOT_LENGTH = new Distance(13, Distance.Unit.INCH);

	public static final Distance FIELD_WIDTH = new Distance(324, Distance.Unit.INCH);

	public static final Distance BASELINE_TO_PLATFORM_ZONE_X = new Distance(236, Distance.Unit.INCH)
			.sub(ROBOT_LENGTH.mul(0.5));
	public static final Distance BASELINE_TO_PLATFORM_ZONE_Y = Distance.ZERO;
	
	public static final Distance BASELINE_TO_SCALE_X = new Distance(266, Distance.Unit.INCH).add(new Distance(6, Distance.Unit.INCH));

	public static final Distance BASELINE_TO_MID_SWITCH_X = new Distance(168, Distance.Unit.INCH).sub(ROBOT_LENGTH.mul(0.5));
	
	public static final Distance BASELINE_TO_SWITCH_X = new Distance(140, Distance.Unit.INCH).sub(ROBOT_LENGTH);

	public static final Distance BASELINE_TO_10_CUBE_PILE = new Distance(98, Distance.Unit.INCH).sub(ROBOT_LENGTH);
	
	public static final Distance SWITCH_TO_PIVOT_POINT_X = new Distance(88, Distance.Unit.INCH);

	public static final Distance PIVOT_POINT_FIELD_EDGE_Y = new Distance(54, Distance.Unit.INCH); //54 for outside

	public static final Distance SWITCH_EDGE_TO_FIELD_EDGE_Y = new Distance(85.25, Distance.Unit.INCH);

	public static final Distance PLATFORM_ZONE_WIDTH_START_POS = new Distance(259, Distance.Unit.INCH).sub(ROBOT_WIDTH);
	
	//Change 18 to be half of cube to platform zone diagonal first number
	public static final Distance PLATFORM_ZONE_WIDTH_BLOCK = FIELD_WIDTH.sub(SWITCH_EDGE_TO_FIELD_EDGE_Y.mul(2))
			.add(new Distance(18, Distance.Unit.INCH)).add(INTAKE_LENGTH.mul(0.5).add(ROBOT_LENGTH.mul(0.25)));
	//TODO: Find more precisely
	
	public static final Distance FIELD_EDGE_TO_CUBE_2_Y = new Distance(113, Distance.Unit.INCH);
	public static final Distance FIELD_EDGE_TO_CUBE_3_Y = new Distance(142, Distance.Unit.INCH);
	
	public static final Distance CUBE_1_TO_PIVOT_POINT_DIAGONAL = NRMath
			.hypot(SWITCH_EDGE_TO_FIELD_EDGE_Y.sub(PIVOT_POINT_FIELD_EDGE_Y).add(BLOCK_WIDTH.mul(0.5)),
					new Distance(61, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5)))
			.sub(ROBOT_LENGTH.mul(0.5)).sub(INTAKE_LENGTH).add(new Distance(10, Distance.Unit.INCH));

	public static final Distance CUBE_2_TO_PIVOT_POINT_DIAGONAL = NRMath
			.hypot(FIELD_EDGE_TO_CUBE_2_Y.sub(PIVOT_POINT_FIELD_EDGE_Y).add(BLOCK_WIDTH.mul(0.5)),
					new Distance(61, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5)))
			.sub(ROBOT_LENGTH.mul(0.5)).sub(INTAKE_LENGTH).add(new Distance(6, Distance.Unit.INCH));

	public static final Distance CUBE_3_TO_PIVOT_POINT_DIAGONAL = NRMath
			.hypot(FIELD_EDGE_TO_CUBE_3_Y.sub(PIVOT_POINT_FIELD_EDGE_Y).add(BLOCK_WIDTH.mul(0.5)),
					new Distance(61, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5)))
			.sub(ROBOT_LENGTH.mul(0.5)).sub(INTAKE_LENGTH).add(new Distance(6, Distance.Unit.INCH));

	////////////////////////////////////////
	//Distances For Middle Start Position
	public static final Distance PUSH_OFF_WALL_X = new Distance(6, Distance.Unit.INCH);
	public static final Distance START_POS_MID_TO_SWITCH_LEFT_DIAGONAL = NRMath
			.hypot(BASELINE_TO_10_CUBE_PILE.sub(PUSH_OFF_WALL_X).add(ROBOT_LENGTH.mul(0.5)), new Distance(9, Distance.Unit.FOOT));
	
	public static final Distance FRONT_SWITCH_LOOP_TO_CUBE_MID_DIAGONAL = NRMath
			.hypot(new Distance(41, Distance.Unit.INCH), new Distance(15, Distance.Unit.INCH))
			.sub(ROBOT_LENGTH.mul(0.5)).sub(INTAKE_LENGTH.mul(0.5));
	public static final Distance FRONT_SWITCH_LOOP_TO_CUBE_FAR_DIAGONAL = NRMath
			.hypot(new Distance(35, Distance.Unit.INCH), new Distance(28, Distance.Unit.INCH))
			.sub(ROBOT_LENGTH.mul(0.5)).sub(INTAKE_LENGTH.mul(0.5));
	////////////////////////////////////////
	
	
	////////////////////////////////////////
	//Angles For Middle Start Position
	public static final Angle START_POS_MID_TO_SWITCH_ANGLE_LEFT = new Angle(Math.atan(
			 new Distance(9, Distance.Unit.FOOT).div(
					 BASELINE_TO_10_CUBE_PILE.sub(PUSH_OFF_WALL_X).add(ROBOT_LENGTH.mul(0.5)))),
			Angle.Unit.RADIAN);
	
	public static final Angle FRONT_SWITCH_LOOP_TO_CUBE_MID_ANGLE = new Angle(Math.atan(new Distance(41, Distance.Unit.INCH).div
			(new Distance(15, Distance.Unit.INCH))), Angle.Unit.RADIAN);
	public static final Angle FRONT_SWITCH_LOOP_TO_CUBE_FAR_ANGLE = new Angle(Math.atan(new Distance(35, Distance.Unit.INCH).div
			(new Distance(28, Distance.Unit.INCH))), Angle.Unit.RADIAN);
	////////////////////////////////////////
	
	public static final Distance CUBE_TO_PLATFORM_ZONE_DIAGONAL = new Distance(36, Distance.Unit.INCH).sub(BLOCK_WIDTH.mul(0.5));
	
	public static final Distance PLATFORM_ZONE_TO_SCALE_DIAGONAL = new Distance(44, Distance.Unit.INCH);
	
	// Platform zone Y direction to diagonal of cube
	public static final Angle PIVOT_POINT_TO_CUBE_1 = new Angle(
			Math.atan((new Distance(61, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5)))
					.div((SWITCH_EDGE_TO_FIELD_EDGE_Y.sub(PIVOT_POINT_FIELD_EDGE_Y).add(BLOCK_WIDTH.mul(0.5))))),
			Angle.Unit.RADIAN).add(new Angle(10, Angle.Unit.DEGREE)).add(new Angle(10, Angle.Unit.DEGREE));
	public static final Angle PIVOT_POINT_TO_CUBE_2 = new Angle(
			Math.atan((new Distance(61, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5)))
					.div((FIELD_EDGE_TO_CUBE_2_Y.sub(PIVOT_POINT_FIELD_EDGE_Y).add(BLOCK_WIDTH.mul(0.5))))),
			Angle.Unit.RADIAN);
	public static final Angle PIVOT_POINT_TO_CUBE_3 = new Angle(
			Math.atan((new Distance(61, Distance.Unit.INCH).add(BLOCK_WIDTH.mul(0.5))
					.div((FIELD_EDGE_TO_CUBE_3_Y.sub(PIVOT_POINT_FIELD_EDGE_Y).add(BLOCK_WIDTH.mul(0.5)))))),
			Angle.Unit.RADIAN);

	// Platform Zone Y direction to diagonal of scale
	public static final Angle PIVOT_POINT_TO_SCALE = new Angle(52, Angle.Unit.DEGREE);
	public static final Angle PIVOT_POINT_TO_SCALE_ACROSS_FIELD = new Angle(65, Angle.Unit.DEGREE);
	
	//For switch strafing
	public static final Distance CUBE_1_AND_2_TO_SWITCH_Y = ROBOT_WIDTH.mul(0.5);
	public static final Distance CUBE_3_TO_SWITCH_Y = new Distance(28, Distance.Unit.INCH).add(ROBOT_WIDTH.mul(0.5));
	public static final Distance CUBE_TO_PLATFORM_ZONE_X = new Distance(27, Distance.Unit.INCH).sub(ROBOT_LENGTH.mul(0.5));
	
	public static final Angle SWITCH_LOOP_SWITCH_ANGLE = new Angle(10, Angle.Unit.DEGREE);
	
	public static final Distance CUBE_1_TO_SWITCH_DIAGONAL = new Distance(16, Distance.Unit.INCH);
	
	public static final Angle CUBE_1_TO_CUBE_2 = PIVOT_POINT_TO_CUBE_1.sub(PIVOT_POINT_TO_CUBE_2);
}
