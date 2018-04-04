package edu.nr.robotics;

import edu.nr.lib.commandbased.CancelAllCommand;
import edu.nr.lib.commandbased.DoNothingCommand;
import edu.nr.lib.gyro.ResetGyroCommand;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.multicommands.ClimbButtonCommand;
import edu.nr.robotics.multicommands.ClimbPrepareCommand;
import edu.nr.robotics.multicommands.ClimberStopButtonCommand;
import edu.nr.robotics.multicommands.CubeFeedIntakeRollersToElevatorCommand;
import edu.nr.robotics.multicommands.CubeFeedIntakeRollersToElevatorCommandManual;
import edu.nr.robotics.multicommands.FoldIntakeMultiCommand;
import edu.nr.robotics.multicommands.PrepareScoreElevatorBottomCommand;
import edu.nr.robotics.multicommands.PrepareScorePortalCommand;
import edu.nr.robotics.multicommands.PrepareScoreSwitchCommand;
import edu.nr.robotics.subsystems.climber.ClimberCoastCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveDumbToggleCommand;
import edu.nr.robotics.subsystems.drive.DriveToCubeButtonCommand;
import edu.nr.robotics.subsystems.drive.DriveToCubeJoystickCommand;
import edu.nr.robotics.subsystems.drive.EnableSniperModeCommand;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileDeltaCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterShootCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeDeployCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorBottomCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersReverseCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersStopCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersVelocityCommand;
import edu.nr.robotics.subsystems.sensors.EnableFloorSensorCommand;
import edu.nr.robotics.subsystems.sensors.EnableLimelightCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.ToggleLimelightCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements SmartDashboardSource {

	public static final double JOYSTICK_DEAD_ZONE = 0.15;

	// TODO: Find button numbers
	private static final int CANCEL_ALL_BUTTON_NUMBER = 1; // Right!
	private static final int FOLD_INTAKE_BUTTON_NUMBER = 11; // Right
	private static final int INTAKE_TRANSFER_HEIGHT_BUTTON_NUMBER = 10; // Right
	private static final int INTAKE_BOTTOM_HEIGHT_BUTTON_NUMBER = 9; // Right

	private static final int TRANSFER_CUBE_BUTTON_NUMBER = 4; //Right
	
	private static final int INTAKE_ROLLERS_STOP_BUTTON_NUMBER = 2; // Right
	private static final int INTAKE_ROLLERS_FAST_BUTTON_NUMBER = 11; // Left
	private static final int INTAKE_ROLLERS_SLOW_BUTTON_NUMBER = 8; // Left

	private static final int ELEVATOR_CLIMB_HEIGHT_BUTTON_NUMBER = 6; // Right
	private static final int ELEVATOR_BOTTOM_HEIGHT_BUTTON_NUMBER = 8; // Right

	private static final int SCALE_BUTTON_NUMBER = 5; // Left
	private static final int SWITCH_BUTTON_NUMBER = 6; // Left
	private static final int BOTTOM_BUTTON_NUMBER = 4; // Left
	private static final int LOW_GOAL_BUTTON_NUMBER = 7; // Left

	private static final int SHOOT_CUBE_BUTTON_NUMBER = 12; // Left
	private static final int PLACE_CUBE_BUTTON_NUMBER = 9; // Left

	private static final int RUN_INTAKE_BUTTON_NUMBER = 11; //Right drive
	
	private static final int ELEVATOR_UP_BUTTON_NUMBER = 5; // Right
	private static final int ELEVATOR_DOWN_BUTTON_NUMBER = 3; // Right

	private static final int CLIMB_BUTTON_NUMBER = 10; // Left

	private static final int ACQUIRE_CUBE_BUTTON_NUMBER = 12; // Right
	private static final int HALF_TURN_BUTTON_NUMBER = 3;
	private static final int QUARTER_TURN_LEFT_BUTTON_NUMBER = 4;
	private static final int QUARTER_TURN_RIGHT_BUTTON_NUMBER = 5;
	private static final int DUMB_DRIVE_BUTTON_NUMBER = 10;

	private static final int ENABLE_SCALE_STOPPING_BUTTON_NUMBER = 1;
	private static final int DRIVE_TO_CUBE_JOYSTICK_BUTTON_NUMBER = 5;
	
	private static final int RESET_GYRO_BUTTON_NUMBER = 7;
	private static final int CLIMB_COAST_BUTTON_NUMBER = 6;

	private static final int ENABLE_LIMELIGHT_BUTTON = 6;
	
	private static final int SNIPER_MODE_BUTTON = 1;
	
	private static final int TEST_SEQUENCE_BUTTON_NUMBER = 10;

	private double driveSpeedMultiplier = 1;

	private static OI singleton;

	private final Joystick driveLeft;
	private final Joystick driveRight;

	private final Joystick operatorLeft;
	private final Joystick operatorRight;

	private final Joystick elevatorStick;
	private final Joystick intakeElevatorStick;

	private JoystickButton scaleButton;
	private JoystickButton switchButton;
	private JoystickButton bottomButton;
	private JoystickButton lowGoalButton;

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

		elevatorStick = operatorRight;
		intakeElevatorStick = operatorLeft;

		initDriveLeft();
		initDriveRight();

		initOperatorLeft();
		initOperatorRight();

		SmartDashboardSource.sources.add(this);
	}

	public void initDriveLeft() {

		new JoystickButton(driveLeft, DUMB_DRIVE_BUTTON_NUMBER).whenPressed(new DriveDumbToggleCommand());

		new JoystickButton(driveLeft, ENABLE_SCALE_STOPPING_BUTTON_NUMBER)
				.whenPressed(new EnableFloorSensorCommand(true));
		new JoystickButton(driveLeft, ENABLE_SCALE_STOPPING_BUTTON_NUMBER)
				.whenReleased(new EnableFloorSensorCommand(false));

		// new JoystickButton(driveLeft,
		// TEST_SEQUENCE_BUTTON_NUMBER).whenPressed(new
		// SystemTestSequenceCommand());

		new JoystickButton(driveLeft, CLIMB_COAST_BUTTON_NUMBER).whenPressed(new ClimberCoastCommand(true));
		
		new JoystickButton(driveLeft, DRIVE_TO_CUBE_JOYSTICK_BUTTON_NUMBER).whenPressed(new DriveToCubeJoystickCommand());
		new JoystickButton(driveLeft, DRIVE_TO_CUBE_JOYSTICK_BUTTON_NUMBER).whenReleased(new DoNothingCommand(Drive.getInstance()));
	}

	public void initDriveRight() {

		new JoystickButton(driveRight, RESET_GYRO_BUTTON_NUMBER).whenPressed(new ResetGyroCommand());

		new JoystickButton(driveRight, HALF_TURN_BUTTON_NUMBER).whenPressed(new TurnCommand(Drive.getInstance(),
				new Angle(180, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));
		new JoystickButton(driveRight, QUARTER_TURN_LEFT_BUTTON_NUMBER).whenPressed(new TurnCommand(Drive.getInstance(),
				new Angle(-90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));
		new JoystickButton(driveRight, QUARTER_TURN_RIGHT_BUTTON_NUMBER).whenPressed(
				new TurnCommand(Drive.getInstance(), new Angle(90, Angle.Unit.DEGREE), Drive.MAX_PROFILE_TURN_PERCENT));
	
		new JoystickButton(driveRight, ENABLE_LIMELIGHT_BUTTON).whenPressed(new ToggleLimelightCommand());
	
		new JoystickButton(driveRight, RUN_INTAKE_BUTTON_NUMBER).whenPressed(new IntakeRollersVelocityCommand(IntakeRollers.VEL_PERCENT_HIGH_INTAKE_ROLLERS, IntakeRollers.VEL_PERCENT_LOW_INTAKE_ROLLERS));
		new JoystickButton(driveRight, RUN_INTAKE_BUTTON_NUMBER).whenReleased(new IntakeRollersStopCommand());
		
		new JoystickButton(driveRight, SNIPER_MODE_BUTTON).whenPressed(new EnableSniperModeCommand(true));
		new JoystickButton(driveRight, SNIPER_MODE_BUTTON).whenReleased(new EnableSniperModeCommand(false));
	}

	public void initOperatorLeft() {

		new JoystickButton(operatorLeft, INTAKE_ROLLERS_FAST_BUTTON_NUMBER)
				.whenPressed(new IntakeRollersReverseCommand(IntakeRollers.VEL_PERCENT_PORTAL_INTAKE_ROLLERS));
		new JoystickButton(operatorLeft, INTAKE_ROLLERS_SLOW_BUTTON_NUMBER)
				.whenPressed(new IntakeRollersReverseCommand(IntakeRollers.VEL_PERCENT_PUKE_INTAKE_ROLLERS));

		new JoystickButton(operatorLeft, SHOOT_CUBE_BUTTON_NUMBER)
				.whenPressed(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_HIGH_ELEVATOR_SHOOTER));
		new JoystickButton(operatorLeft, PLACE_CUBE_BUTTON_NUMBER)
				.whenPressed(new ElevatorShooterShootCommand(ElevatorShooter.VEL_PERCENT_LOW_ELEVATOR_SHOOTER));

		new JoystickButton(operatorLeft, CLIMB_BUTTON_NUMBER).whenPressed(new ClimbButtonCommand());
		new JoystickButton(operatorLeft, CLIMB_BUTTON_NUMBER).whenReleased(new ClimberStopButtonCommand());

		scaleButton = new JoystickButton(operatorLeft, SCALE_BUTTON_NUMBER);
		switchButton = new JoystickButton(operatorLeft, SWITCH_BUTTON_NUMBER);
		bottomButton = new JoystickButton(operatorLeft, BOTTOM_BUTTON_NUMBER);
		lowGoalButton = new JoystickButton(operatorLeft, LOW_GOAL_BUTTON_NUMBER);

		//Scale
		scaleButton.whenPressed(new ElevatorProfileCommandGroup(Elevator.SCALE_HEIGHT_ELEVATOR,
				Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
		
		//Switch
		switchButton.whenPressed(new PrepareScoreSwitchCommand());
		
		//Bottom
		bottomButton.whenPressed(new PrepareScoreElevatorBottomCommand());
		
		//Low Goal
		lowGoalButton.whenPressed(new PrepareScorePortalCommand());

	}

	public void initOperatorRight() {

		new JoystickButton(operatorRight, FOLD_INTAKE_BUTTON_NUMBER).whenPressed(new FoldIntakeMultiCommand());
		new JoystickButton(operatorRight, INTAKE_TRANSFER_HEIGHT_BUTTON_NUMBER)
				.whenPressed(new IntakeElevatorProfileCommandGroup(IntakeElevator.HANDLER_HEIGHT, IntakeElevator.PROFILE_VEL_PERCENT_INTAKE_ELEVATOR, IntakeElevator.PROFILE_ACCEL_PERCENT_INTAKE_ELEVATOR));
		new JoystickButton(operatorRight, INTAKE_BOTTOM_HEIGHT_BUTTON_NUMBER)
				.whenPressed(new IntakeDeployCommand());


		new JoystickButton(operatorRight, CANCEL_ALL_BUTTON_NUMBER).whenPressed(new CancelAllCommand());

		new JoystickButton(operatorRight, ACQUIRE_CUBE_BUTTON_NUMBER).whenPressed(new DriveToCubeButtonCommand(true));
		
		new JoystickButton(operatorRight, INTAKE_ROLLERS_STOP_BUTTON_NUMBER)
				.whenPressed(new IntakeRollersStopCommand());

		// new JoystickButton(operatorRight,
		// ELEVATOR_UP_BUTTON_NUMBER).whenPressed(new
		// ElevatorProfileCommandGroup(ELEVATOR_BUTTON_HEIGHT,
		// Elevator.PROFILE_VEL_PERCENT_ELEVATOR,
		// Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
		// new JoystickButton(operatorRight,
		// ELEVATOR_DOWN_BUTTON_NUMBER).whenPressed(new
		// ElevatorProfileCommandGroup(ELEVATOR_BUTTON_HEIGHT.negate(),
		// Elevator.PROFILE_VEL_PERCENT_ELEVATOR,
		// Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));

		new JoystickButton(operatorRight, ELEVATOR_UP_BUTTON_NUMBER)
				.whenPressed(new ElevatorProfileDeltaCommandGroup(ELEVATOR_BUTTON_HEIGHT, Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));
		new JoystickButton(operatorRight, ELEVATOR_DOWN_BUTTON_NUMBER)
				.whenPressed(new ElevatorProfileDeltaCommandGroup(ELEVATOR_BUTTON_HEIGHT.negate(), Elevator.PROFILE_VEL_PERCENT_ELEVATOR, Elevator.PROFILE_ACCEL_PERCENT_ELEVATOR));

		new JoystickButton(operatorRight, ELEVATOR_CLIMB_HEIGHT_BUTTON_NUMBER)
				.whenPressed(new ClimbPrepareCommand());
		
		new JoystickButton(operatorRight, ELEVATOR_BOTTOM_HEIGHT_BUTTON_NUMBER).whenPressed(new ElevatorBottomCommand());

		new JoystickButton(operatorRight, TRANSFER_CUBE_BUTTON_NUMBER).whenPressed(new CubeFeedIntakeRollersToElevatorCommandManual());
		
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
		return -snapDriveJoysticks(driveLeft.getY());
	}

	public double getArcadeTurnValue() {
		return -snapDriveJoysticks(driveRight.getX()) * Drive.TURN_JOYSTICK_MULTIPLIER;
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
		SmartDashboard.putBoolean("Intake Sensor", !EnabledSensors.intakeSensorLeft.get() && !EnabledSensors.intakeSensorRight.get());
		SmartDashboard.putBoolean("Floor Sensor", !EnabledSensors.floorSensor.get());
		SmartDashboard.putBoolean("Elevator Sensor", !EnabledSensors.elevatorSensor.get());
	}

	public boolean isTankNonZero() {
		return getTankLeftValue() != 0 || getTankRightValue() != 0 || getTankHValue() != 0;
	}

	public boolean isArcadeNonZero() {
		return getArcadeMoveValue() != 0 || getArcadeTurnValue() != 0 || getArcadeHValue() != 0;
	}

	public boolean isDriveNonZero() {
		return getDriveLeftXValue() != 0 || getDriveRightXValue() != 0 || getDriveLeftYValue() != 0
				|| getDriveRightYValue() != 0;
	}

	public boolean isElevatorNonZero() {
		return getElevatorJoystickValue() != 0;
	}

	public boolean isIntakeElevatorNonZero() {
		return getIntakeElevatorJoystickValue() != 0;
	}

	public boolean isHDriveZero() {
		return snapDriveJoysticks(driveLeft.getX()) == 0;
	}
}
