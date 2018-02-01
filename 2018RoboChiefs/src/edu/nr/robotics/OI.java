package edu.nr.robotics;

import edu.nr.lib.commandbased.CancelAllCommand;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.joystickbuttons.ConditionalDoubleJoystickButton;
import edu.nr.lib.joystickbuttons.ConditionalJoystickButton;
import edu.nr.lib.joystickbuttons.DoubleJoystickButton;
import edu.nr.lib.joystickbuttons.DoubleJoystickButton.Type;
import edu.nr.lib.units.Angle;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.multicommands.DriveToCubeCommandAdvanced;
import edu.nr.robotics.multicommands.PrepareScorePortalCommand;
import edu.nr.robotics.multicommands.PrepareScoreScaleCommand;
import edu.nr.robotics.multicommands.PrepareScoreSwitchCommand;
import edu.nr.robotics.multicommands.ScorePortalCommand;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandler;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandlerStopCommand;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandlerVelocityCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionCommand;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ShootCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorPositionCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersStopCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersVelocityCommand;
import edu.nr.robotics.subsystems.sensors.EnableFloorSensorCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements SmartDashboardSource {
	
	private static final double JOYSTICK_DEAD_ZONE = 0.15;
	
	//TODO: Find button numbers
	private static final int CANCEL_ALL_BUTTON_NUMBER = 1;
	private static final int FOLD_INTAKE_BUTTON_NUMBER = 5;
	private static final int INTAKE_HANDLER_HEIGHT_BUTTON_NUMBER = 2;
	private static final int INTAKE_BOTTOM_HEIGHT_BUTTON_NUMBER = 3;
	private static final int CUBE_HANDLER_BUTTON_NUMBER = 4;
	private static final int INTAKE_ROLLERS_BUTTON_NUMBER = 5;
	private static final int INTAKE_TO_ELEVATOR_BUTTON_NUMBER = 6;
	private static final int INTAKE_TO_PORTAL_BUTTON_NUMBER = 7;
	private static final int ELEVATOR_CLIMB_HEIGHT_BUTTON_NUMBER = 8;
	private static final int ELEVATOR_SCALE_HEIGHT_BUTTON_NUMBER = 9;
	private static final int ELEVATOR_SWITCH_HEIGHT_BUTTON_NUMBER = 10;
	private static final int ELEVATOR_BOTTOM_HEIGHT_BUTTON_NUMBER = 11;
	private static final int SHOOT_CUBE_BUTTON_NUMBER = 3;
	private static final int PLACE_CUBE_BUTTON_NUMBER = 4;
	private static final int ACQUIRE_CUBE_BUTTON_NUMBER = 12;
	private static final int SCORE_IN_PORTAL_BUTTON_NUMBER = 1;
	private static final int ENABLE_SCALE_STOPPING_BUTTON_NUMBER = 1;
	
	private static final int HALF_TURN_BUTTON_NUMBER = 2;
	private static final int QUARTER_TURN_LEFT_BUTTON_NUMBER = 3;
	private static final int QUARTER_TURN_RIGHT_BUTTON_NUMBER = 4;
	//private static final int CLIMB_BUTTON_NUMBER = -1;
	//TODO: Find how we want to control elevator
	
	private double driveSpeedMultiplier = 1;
	
	private Direction portalStrafeDirection = Direction.left;
	
	private static OI singleton;

	private final Joystick driveLeft;
	private final Joystick driveRight;
	
	private final Joystick operatorLeft;
	private final Joystick operatorRight;
	
	private final Joystick elevatorStick;
	
	private JoystickButton intakeToElevatorButton;
	private JoystickButton elevatorScaleHeightButton;
	private JoystickButton elevatorSwitchHeightButton;
		
	private static final int STICK_LEFT = 0;
	private static final int STICK_RIGHT = 1;
	private static final int STICK_OPERATOR_LEFT = 2;
	private static final int STICK_OPERATOR_RIGHT = 3;
	
	public static final Drive.DriveMode driveMode = Drive.DriveMode.cheesyDrive;
	
	private OI() {		
		driveLeft = new Joystick(STICK_LEFT);
		driveRight = new Joystick(STICK_RIGHT);
		
		operatorLeft = new Joystick(STICK_OPERATOR_LEFT);
		operatorRight = new Joystick(STICK_OPERATOR_RIGHT);

		elevatorStick = operatorRight;
		
		initDriveLeft();
		initDriveRight();
		
		initOperatorLeft();
		initOperatorRight();
		
		SmartDashboardSource.sources.add(this);
	}
	
	public void initDriveLeft() {
		new JoystickButton(driveLeft, SCORE_IN_PORTAL_BUTTON_NUMBER).whenPressed(new ScorePortalCommand(portalStrafeDirection));
		
		new JoystickButton(driveLeft, ENABLE_SCALE_STOPPING_BUTTON_NUMBER).whenPressed(new EnableFloorSensorCommand(true));
		new JoystickButton(driveLeft, ENABLE_SCALE_STOPPING_BUTTON_NUMBER).whenReleased(new EnableFloorSensorCommand(false));
		
		new JoystickButton(operatorLeft, SHOOT_CUBE_BUTTON_NUMBER).whenPressed(new ShootCommand(ElevatorShooter.VEL_PERCENT_HIGH_ELEVATOR_SHOOTER));
		new JoystickButton(operatorLeft, PLACE_CUBE_BUTTON_NUMBER).whenPressed(new ShootCommand(ElevatorShooter.VEL_PERCENT_LOW_ELEVATOR_SHOOTER));
	
		new JoystickButton(operatorLeft, FOLD_INTAKE_BUTTON_NUMBER).whenPressed(new IntakeElevatorPositionCommand(IntakeElevator.FOLDED_HEIGHT));	
	}
	
	public void initDriveRight() {
		
		new JoystickButton(driveRight, HALF_TURN_BUTTON_NUMBER).whenPressed(new TurnCommand(Drive.getInstance(), new Angle(180, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, false));
		new JoystickButton(driveRight, QUARTER_TURN_LEFT_BUTTON_NUMBER).whenPressed(new TurnCommand(Drive.getInstance(), new Angle(-90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, false));
		new JoystickButton(driveRight, QUARTER_TURN_RIGHT_BUTTON_NUMBER).whenPressed(new TurnCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT, false));
	}
	
	public void initOperatorLeft() {
	
		intakeToElevatorButton = new JoystickButton(operatorLeft, INTAKE_TO_ELEVATOR_BUTTON_NUMBER);
		elevatorScaleHeightButton = new JoystickButton(operatorLeft, ELEVATOR_SCALE_HEIGHT_BUTTON_NUMBER);
		elevatorSwitchHeightButton = new JoystickButton(operatorLeft, ELEVATOR_SWITCH_HEIGHT_BUTTON_NUMBER);
		
		
		new JoystickButton(operatorLeft, INTAKE_HANDLER_HEIGHT_BUTTON_NUMBER).whenPressed(new IntakeElevatorPositionCommand(IntakeElevator.HANDLER_HEIGHT));
		new JoystickButton(operatorLeft, INTAKE_BOTTOM_HEIGHT_BUTTON_NUMBER).whenPressed(new IntakeElevatorPositionCommand(IntakeElevator.INTAKE_HEIGHT));
		
		new JoystickButton(operatorLeft, CUBE_HANDLER_BUTTON_NUMBER).whenPressed(new CubeHandlerVelocityCommand(CubeHandler.VEL_PERCENT_CUBE_HANDLER));
		new JoystickButton(operatorLeft, CUBE_HANDLER_BUTTON_NUMBER).whenReleased(new CubeHandlerStopCommand());
		
		new JoystickButton(operatorLeft, INTAKE_ROLLERS_BUTTON_NUMBER).whenPressed(new IntakeRollersVelocityCommand(IntakeRollers.VEL_PERCENT_INTAKE_ROLLERS));
		new JoystickButton(operatorLeft, INTAKE_ROLLERS_BUTTON_NUMBER).whenReleased(new IntakeRollersStopCommand());
		
		new ConditionalDoubleJoystickButton(intakeToElevatorButton, elevatorScaleHeightButton, Type.And,
				!EnabledSensors.intakeSensor.get()).whenPressed(new PrepareScoreScaleCommand());;
		new ConditionalDoubleJoystickButton(intakeToElevatorButton, elevatorSwitchHeightButton, Type.And,
				!EnabledSensors.intakeSensor.get()).whenPressed(new PrepareScoreSwitchCommand());
		
		new ConditionalJoystickButton(new JoystickButton(operatorLeft, INTAKE_TO_PORTAL_BUTTON_NUMBER), 
				!EnabledSensors.intakeSensor.get()).whenPressed(new PrepareScorePortalCommand());
		
		new JoystickButton(operatorLeft, ELEVATOR_CLIMB_HEIGHT_BUTTON_NUMBER).whenPressed(new ElevatorPositionCommand(Elevator.CLIMB_HEIGHT_ELEVATOR));
		new ConditionalJoystickButton(elevatorScaleHeightButton, !intakeToElevatorButton.get()).whenPressed(new ElevatorPositionCommand(Elevator.SCALE_HEIGHT_ELEVATOR));
		new ConditionalJoystickButton(elevatorSwitchHeightButton, !intakeToElevatorButton.get()).whenPressed(new ElevatorPositionCommand(Elevator.SWITCH_HEIGHT_ELEVATOR));
		new JoystickButton(operatorLeft, ELEVATOR_BOTTOM_HEIGHT_BUTTON_NUMBER).whenPressed(new ElevatorPositionCommand(Elevator.BOTTOM_HEIGHT_ELEVATOR));
		
		
	}
	
	public void initOperatorRight() {
		new JoystickButton(operatorRight, ACQUIRE_CUBE_BUTTON_NUMBER).whenPressed(new DriveToCubeCommandAdvanced());
		
		new JoystickButton(operatorRight, CANCEL_ALL_BUTTON_NUMBER).whenPressed(new CancelAllCommand());
		
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
		return snapDriveJoysticks(elevatorStick.getY());
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
	
	public boolean shouldDumbDrive() {
		//TODO: Find how we want to switch to dumb drive
		return false;
	}
}
