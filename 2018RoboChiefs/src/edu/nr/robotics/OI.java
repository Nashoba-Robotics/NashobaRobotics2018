package edu.nr.robotics;

import edu.nr.lib.commandbased.CancelAllCommand;
import edu.nr.lib.gyro.ResetGyroCommand;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.multicommands.ClimbButtonCommand;
import edu.nr.robotics.multicommands.CubeFeedIntakeRollersToElevatorCommand;
import edu.nr.robotics.multicommands.DriveToCubeButtonCommand;
import edu.nr.robotics.multicommands.PrepareScorePortalCommand;
import edu.nr.robotics.subsystems.climber.ClimberCoastCommand;
import edu.nr.robotics.subsystems.climber.ClimberStopCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveDumbToggleCommand;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomDropCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterShootCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorFoldCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorHandlerCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorMoveBasicCommand;
import edu.nr.robotics.subsystems.sensors.EnableFloorSensorCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements SmartDashboardSource {
	
	public static final double JOYSTICK_DEAD_ZONE = 0.15;
	
	//TODO: Find button numbers
	private static final int CANCEL_ALL_BUTTON_NUMBER = 1;
	private static final int FOLD_INTAKE_BUTTON_NUMBER = 4;
	private static final int INTAKE_HANDLER_HEIGHT_BUTTON_NUMBER = 9;
	private static final int INTAKE_BOTTOM_HEIGHT_BUTTON_NUMBER = 6;
	
	private static final int INTAKE_ROLLERS_BUTTON_NUMBER = -1;
	private static final int INTAKE_ROLLERS_STOP_BUTTON_NUMBER = -1;
	private static final int INTAKE_ROLLERS_PORTAL_BUTTON_NUMBER = -1;
	private static final int INTAKE_ROLLERS_PUKE_BUTTON_NUMBER = -1;
	
	private static final int INTAKE_TO_ELEVATOR_BUTTON_NUMBER = 7;
	private static final int ELEVATOR_CLIMB_HEIGHT_BUTTON_NUMBER = 6;
	private static final int ELEVATOR_SCALE_HEIGHT_BUTTON_NUMBER = 5;
	private static final int ELEVATOR_SWITCH_HEIGHT_BUTTON_NUMBER = 3;
	private static final int ELEVATOR_BOTTOM_HEIGHT_BUTTON_NUMBER = 4;
	private static final int SHOOT_CUBE_BUTTON_NUMBER = 10;
	private static final int PLACE_CUBE_BUTTON_NUMBER = 7;
	
	private static final int ELEVATOR_UP_BUTTON_NUMBER = 11;
	private static final int ELEVATOR_DOWN_BUTTON_NUMBER = 10;
	
	private static final int ACQUIRE_CUBE_BUTTON_NUMBER = 12;
	private static final int HALF_TURN_BUTTON_NUMBER = 2;
	private static final int QUARTER_TURN_LEFT_BUTTON_NUMBER = 3;
	private static final int QUARTER_TURN_RIGHT_BUTTON_NUMBER = 4;
	private static final int DUMB_DRIVE_BUTTON_NUMBER = 10;
	
	private static final int ENABLE_SCALE_STOPPING_BUTTON_NUMBER = 1;
	private static final int RESET_GYRO_BUTTON_NUMBER = 8;
	private static final int CLIMB_BUTTON_NUMBER = 2;
	private static final int CLIMB_COAST_BUTTON_NUMBER= 10;
	
	private static final int TEST_SEQUENCE_BUTTON_NUMBER = -1;
	
	private double driveSpeedMultiplier = 1;
		
	private static OI singleton;

	private final Joystick driveLeft;
	private final Joystick driveRight;
	
	private final Joystick operatorLeft;
	private final Joystick operatorRight;
	
	private final Joystick elevatorStick;
	private final Joystick intakeElevatorStick;
	
	private JoystickButton intakeToElevatorButton;
	private JoystickButton elevatorScaleHeightButton;
	private JoystickButton elevatorSwitchHeightButton;
		
	private static final int STICK_LEFT = 0;
	private static final int STICK_RIGHT = 1;
	private static final int STICK_OPERATOR_LEFT = 2;
	private static final int STICK_OPERATOR_RIGHT = 3;
	
	public static final Distance ELEVATOR_BUTTON_HEIGHT = new Distance(13, Distance.Unit.INCH);
	public static final Drive.DriveMode driveMode = Drive.DriveMode.cheesyDrive;
	
	private OI() {		
		driveLeft = new Joystick(STICK_LEFT);
		driveRight = new Joystick(STICK_RIGHT);
		
		operatorLeft = new Joystick(STICK_OPERATOR_LEFT);
		operatorRight = new Joystick(STICK_OPERATOR_RIGHT);

		elevatorStick = operatorRight; //TODO: find what it is
		intakeElevatorStick = operatorLeft;
		
		initDriveLeft();
		initDriveRight();
		
		initOperatorLeft();
		initOperatorRight();
		
		SmartDashboardSource.sources.add(this);
	}
	
	public void initDriveLeft() {
		
		new JoystickButton(driveLeft, DUMB_DRIVE_BUTTON_NUMBER).whenPressed(new DriveDumbToggleCommand());
		
		new JoystickButton(driveLeft, ENABLE_SCALE_STOPPING_BUTTON_NUMBER).whenPressed(new EnableFloorSensorCommand(true));
		new JoystickButton(driveLeft, ENABLE_SCALE_STOPPING_BUTTON_NUMBER).whenReleased(new EnableFloorSensorCommand(false));
		
		//new JoystickButton(driveLeft, TEST_SEQUENCE_BUTTON_NUMBER).whenPressed(new SystemTestSequenceCommand());
		
		new JoystickButton(driveLeft, CLIMB_COAST_BUTTON_NUMBER).whenPressed(new ClimberCoastCommand(true));
	}
	
	public void initDriveRight() {
		
		new JoystickButton(driveRight, RESET_GYRO_BUTTON_NUMBER).whenPressed(new ResetGyroCommand());
		
		new JoystickButton(driveRight, HALF_TURN_BUTTON_NUMBER).whenPressed(new TurnCommand(Drive.getInstance(), new Angle(180, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));
		new JoystickButton(driveRight, QUARTER_TURN_LEFT_BUTTON_NUMBER).whenPressed(new TurnCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));
		new JoystickButton(driveRight, QUARTER_TURN_RIGHT_BUTTON_NUMBER).whenPressed(new TurnCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));
	}
	
	public void initOperatorLeft() {
	
		new JoystickButton(operatorLeft, FOLD_INTAKE_BUTTON_NUMBER).whenPressed(new IntakeElevatorFoldCommand());	
		new JoystickButton(operatorLeft, INTAKE_HANDLER_HEIGHT_BUTTON_NUMBER).whenPressed(new IntakeElevatorHandlerCommand());
		new JoystickButton(operatorLeft, INTAKE_BOTTOM_HEIGHT_BUTTON_NUMBER).whenPressed(new IntakeElevatorBottomCommand());
		//new JoystickButton(operatorLeft, INTAKE_PORTAL_HEIGHT_BUTTON_NUMBER).whenPressed(new PrepareScorePortalCommand());

		/*
		//new JoystickButton(operatorLeft, CUBE_HANDLER_BUTTON_NUMBER).whenPressed(new CubeHandlerVelocityCommand(CubeHandler.VEL_PERCENT_CUBE_HANDLER));
		//new JoystickButton(operatorLeft, CUBE_HANDLER_BUTTON_NUMBER).whenReleased(new CubeHandlerStopCommand());
		
		new JoystickButton(operatorLeft, INTAKE_ROLLERS_BUTTON_NUMBER).whenPressed(new IntakeRollersIntakeCommand());
		new JoystickButton(operatorLeft, INTAKE_ROLLERS_STOP_BUTTON_NUMBER).whenPressed(new IntakeRollersStopCommand());
		new JoystickButton(operatorLeft, INTAKE_ROLLERS_PORTAL_BUTTON_NUMBER).whenPressed(new IntakeRollersReverseCommand(IntakeRollers.VEL_PERCENT_PORTAL_INTAKE_ROLLERS));
		new JoystickButton(operatorLeft, INTAKE_ROLLERS_PUKE_BUTTON_NUMBER).whenPressed(new IntakeRollersReverseCommand(IntakeRollers.VEL_PERCENT_PUKE_INTAKE_ROLLERS));
		
		new ConditionalDoubleJoystickButton(intakeToElevatorButton, elevatorScaleHeightButton, Type.And,
				!EnabledSensors.intakeSensorLeft.get() && !EnabledSensors.intakeSensorRight.get()).whenPressed(new PrepareScoreScaleCommand());;
		new ConditionalDoubleJoystickButton(intakeToElevatorButton, elevatorSwitchHeightButton, Type.And,
				!EnabledSensors.intakeSensorLeft.get() && !EnabledSensors.intakeSensorRight.get()).whenPressed(new PrepareScoreSwitchCommand());
		
		//new ConditionalJoystickButton(new JoystickButton(operatorLeft, INTAKE_TO_PORTAL_BUTTON_NUMBER), 
		//		!EnabledSensors.intakeSensor.get()).whenPressed(new PrepareScorePortalCommand());
		
		new ConditionalJoystickButton(elevatorScaleHeightButton, !intakeToElevatorButton.get()).whenPressed(new ElevatorPositionCommand(Elevator.SCALE_HEIGHT_ELEVATOR));
		new ConditionalJoystickButton(elevatorSwitchHeightButton, !intakeToElevatorButton.get()).whenPressed(new ElevatorPositionCommand(Elevator.SWITCH_HEIGHT_ELEVATOR));
		*/
		
		new JoystickButton(operatorLeft, SHOOT_CUBE_BUTTON_NUMBER).whenPressed(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_HIGH_ELEVATOR_SHOOTER));
		new JoystickButton(operatorLeft, PLACE_CUBE_BUTTON_NUMBER).whenPressed(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_LOW_ELEVATOR_SHOOTER));
		
	}
	
	public void initOperatorRight() {
		new JoystickButton(operatorRight, CANCEL_ALL_BUTTON_NUMBER).whenPressed(new CancelAllCommand());
		
		new JoystickButton(operatorRight, ACQUIRE_CUBE_BUTTON_NUMBER).whenPressed(new DriveToCubeButtonCommand(true));
		
		new JoystickButton(operatorRight, ELEVATOR_UP_BUTTON_NUMBER).whenPressed(new ElevatorProfileCommandGroup(ELEVATOR_BUTTON_HEIGHT, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
		new JoystickButton(operatorRight, ELEVATOR_DOWN_BUTTON_NUMBER).whenPressed(new ElevatorProfileCommandGroup(ELEVATOR_BUTTON_HEIGHT.negate(), Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
		
		new JoystickButton(operatorRight, INTAKE_TO_ELEVATOR_BUTTON_NUMBER).whenPressed(new CubeFeedIntakeRollersToElevatorCommand());
		
		//new JoystickButton(operatorRight, ELEVATOR_UP_BUTTON_NUMBER).whenPressed(new ElevatorDeltaPositionCommand(ELEVATOR_BUTTON_HEIGHT));
		//new JoystickButton(operatorRight, ELEVATOR_DOWN_BUTTON_NUMBER).whenPressed(new ElevatorDeltaPositionCommand(ELEVATOR_BUTTON_HEIGHT.negate()));
		
		new JoystickButton(operatorRight, ELEVATOR_CLIMB_HEIGHT_BUTTON_NUMBER).whenPressed(new ElevatorProfileCommandGroup(Elevator.CLIMB_HEIGHT_ELEVATOR, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
		new JoystickButton(operatorRight, ELEVATOR_BOTTOM_HEIGHT_BUTTON_NUMBER).whenPressed(new ElevatorBottomDropCommand());
		elevatorScaleHeightButton = new JoystickButton(operatorRight, ELEVATOR_SCALE_HEIGHT_BUTTON_NUMBER);
		elevatorSwitchHeightButton = new JoystickButton(operatorRight, ELEVATOR_SWITCH_HEIGHT_BUTTON_NUMBER);
		
		new JoystickButton(operatorRight, CLIMB_BUTTON_NUMBER).whenPressed(new ClimbButtonCommand());
		new JoystickButton(operatorRight, CLIMB_BUTTON_NUMBER).whenReleased(new ClimberStopCommand());
		
	}
	
	public static OI getInstance() {
		init();
		return singleton;
	}
	
	public static void init() {
		if (singleton == null) {
			singleton = new OI();
		}
	}
	
	public double getArcadeMoveValue() {
		return -snapDriveJoysticks(driveLeft.getY()); //* (driveLeft.getRawButton(DRIVE_REVERSE_BUTTON_NUMBER) ? 1 : -1);
	}

	public double getArcadeTurnValue() {
		return -snapDriveJoysticks(driveRight.getX());
	}
	
	public double getArcadeHValue() {
		return snapDriveJoysticks(driveLeft.getX());
	}

	public double getTankLeftValue() {
		return snapDriveJoysticks(driveLeft.getY());
	}

	public double getTankRightValue() {
		return snapDriveJoysticks(driveRight.getY());
	}
	
	public double getTankHValue() {
		return snapDriveJoysticks(driveLeft.getX());
	}

	public double getDriveLeftXValue() {
		return snapDriveJoysticks(driveLeft.getX());
	}

	public double getDriveLeftYValue() {
		return snapDriveJoysticks(driveLeft.getY());
	}
	
	public double getDriveRightXValue() {
		return snapDriveJoysticks(driveRight.getX());
	}

	public double getDriveRightYValue() {
		return snapDriveJoysticks(driveRight.getY());
	}
	
	public double getElevatorJoystickValue() {
		return snapDriveJoysticks(-elevatorStick.getX());
	}
	
	public double getIntakeElevatorJoystickValue() {
		return snapDriveJoysticks(-intakeElevatorStick.getX());
	}
		
	public double getDriveSpeedMultiplier() {
		return driveSpeedMultiplier;
	}
	
	private static double snapDriveJoysticks(double value) {
		if (Math.abs(value) < JOYSTICK_DEAD_ZONE) {
			value = 0;
		} else if (value > 0) {
			value -= JOYSTICK_DEAD_ZONE;
		} else {
			value += JOYSTICK_DEAD_ZONE;
		}
		value /= 1 - JOYSTICK_DEAD_ZONE;
		return value;
	}
	
	public double getRawMove() {
		return driveLeft.getY();
	}
	
	public double getRawTurn() {
		return driveRight.getX();
	}
	
	private double getTurnAdjust() {
		return driveRight.getRawButton(2) ? 0.5 : 1;
	}
	
	@Override
	public void smartDashboardInfo() {

	}
	
	public boolean isTankNonZero() {
		return getTankLeftValue() != 0 || getTankRightValue() != 0 || getTankHValue() != 0;
	}
	
	public boolean isArcadeNonZero() {
		return getArcadeMoveValue() != 0 || getArcadeTurnValue() != 0 || getArcadeHValue() != 0;
	}
	
	public boolean isDriveNonZero() {
		return getDriveLeftXValue() != 0 || getDriveRightXValue() != 0 || getDriveLeftYValue() != 0 || getDriveRightYValue() != 0;
	}
	
	public boolean isElevatorNonZero() {
		return getElevatorJoystickValue() != 0;
	}
	
	public boolean isIntakeElevatorNonZero() {
		return getElevatorJoystickValue() != 0;
	}
	
	public boolean isHDriveZero() {
		return snapDriveJoysticks(driveLeft.getX()) == 0;
	}
}
