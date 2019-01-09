package edu.nr.robotics;


import edu.nr.lib.commandbased.CancelAllCommand;
import edu.nr.lib.commandbased.DoNothingCommand;
import edu.nr.lib.gyro.ResetGyroCommand;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.units.Angle;
import edu.nr.lib.units.Distance;
import edu.nr.robotics.OI;
import edu.nr.robotics.multicommands.ClimbButtonCommand;
import edu.nr.robotics.multicommands.ClimbPrepareCommand;
import edu.nr.robotics.multicommands.ClimberStopButtonCommand;
import edu.nr.robotics.multicommands.CubeFeedIntakeRollersToElevatorCommandManual;
import edu.nr.robotics.multicommands.FoldIntakeMultiCommand;
import edu.nr.robotics.multicommands.PrepareScoreElevatorBottomCommand;
import edu.nr.robotics.multicommands.PrepareScorePortalCommand;
import edu.nr.robotics.multicommands.PrepareScoreScaleCommand;
import edu.nr.robotics.multicommands.PrepareScoreSwitchCommand;
import edu.nr.robotics.subsystems.climber.ClimberCoastCommand;
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.drive.DriveDumbToggleCommand;
import edu.nr.robotics.subsystems.drive.DriveToCubeButtonCommand;
import edu.nr.robotics.subsystems.drive.DriveToCubeJoystickCommand;
import edu.nr.robotics.subsystems.drive.DriveTuningCommand;
import edu.nr.robotics.subsystems.drive.EnableSniperForwardMode;
import edu.nr.robotics.subsystems.drive.EnableSniperTurnMode;
import edu.nr.robotics.subsystems.drive.TurnCommand;
import edu.nr.robotics.subsystems.drive.TurnToAngleCommand;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorBottomCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileDeltaCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterShootCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeDeployCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevator;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorProfileCommandGroup;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersReverseCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersStopCommand;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersVelocityCommand;
import edu.nr.robotics.subsystems.sensors.EnabledSensors;
import edu.nr.robotics.subsystems.sensors.ToggleLimelightCommand;
import edu.nr.robotics.testSequence.TestSequence;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class OI implements SmartDashboardSource  {
public static final double JOYSTICK_DEAD_ZONE = 0.2;
	
	public static final double SPEED_MULTIPLIER = 1.0;

	// TODO: Find button numbers
	private static final int CANCEL_ALL_BUTTON_NUMBER = 1; // Right!
	private static final int FOLD_INTAKE_BUTTON_NUMBER = 11; // Right
	private static final int INTAKE_TRANSFER_HEIGHT_BUTTON_NUMBER = 10; 9// Right
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

	private static final int RUN_INTAKE_BUTTON_NUMBER = 16; //Right drive
	
	private static final int ELEVATOR_UP_BUTTON_NUMBER = 5; // Right
	private static final int ELEVATOR_DOWN_BUTTON_NUMBER = 3; // Right

	private static final int CLIMB_BUTTON_NUMBER = 10; // Left

	private static final int ACQUIRE_CUBE_BUTTON_NUMBER = 12; // Right
	private static final int HALF_TURN_BUTTON_NUMBER = 2;
	private static final int QUARTER_TURN_LEFT_BUTTON_NUMBER = 3;
	private static final int QUARTER_TURN_RIGHT_BUTTON_NUMBER = 4;
	private static final int DUMB_DRIVE_BUTTON_NUMBER = 14;
	private static final int SNIPER_MODE_BUTTON = 1;
	private static final int TURN_TO_ANGLE_COMMAND_BUTTON = 2; //left

	//private static final int ENABLE_SCALE_STOPPING_BUTTON_NUMBER = 1;
	private static final int DRIVE_TO_CUBE_JOYSTICK_BUTTON_NUMBER = 4;
	
	private static final int RESET_GYRO_BUTTON_NUMBER = 10;
	private static final int CLIMB_COAST_BUTTON_NUMBER = 10;

	private static final int ENABLE_LIMELIGHT_BUTTON = 8;
	
	private static final int DRIVE_TUNING_BUTTON_NUMBER = 8;
	
	private static final int TEST_SEQUENCE_BUTTON_NUMBER = 14;
	
	private static final int KID_MODE_SWITCH = 2;

	private double driveSpeedMultiplier = 1;

	private static OI singleton;

	private final Joystick driveLeft;
	private final Joystick driveRight;

	private final Joystick operatorLeft;
	private final Joystick operatorRight;

	private final Joystick elevatorStick;
	private final Joystick intakeElevatorStick;

	private JoystickButton kidModeSwitch;
	private JoystickButton scaleButton;
	private JoystickButton switchButton;
	private JoystickButton bottomButton;
	private JoystickButton lowGoalButton;

	private static final int STICK_LEFT = 0;
	private static final int STICK_RIGHT = 1;
	private static final int STICK_OPERATOR_LEFT = 2;
	private static final int STICK_OPERATOR_RIGHT = 3;

	public static final Distance ELEVATOR_BUTTON_HEIGHT = new Distance(13, Distance.Unit.INCH);
	public static final Drive.DriveMode driveMode = Drive.DriveMode.tankDrive;

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

		/*new JoystickButton(driveLeft, ENABLE_SCALE_STOPPING_BUTTON_NUMBER)
				.whenPressed(new EnableFloorSensorCommand(true));
		new JoystickButton(driveLeft, ENABLE_SCALE_STOPPING_BUTTON_NUMBER)
				.whenReleased(new EnableFloorSensorCommand(false));*/

		new JoystickButton(driveLeft, SNIPER_MODE_BUTTON).whenPressed(new EnableSniperForwardMode(true));
		new JoystickButton(driveLeft, SNIPER_MODE_BUTTON).whenReleased(new EnableSniperForwardMode(false));		
		
		new JoystickButton(driveLeft, CLIMB_COAST_BUTTON_NUMBER).whenPressed(new ClimberCoastCommand(true));
		
		new JoystickButton(driveLeft, DRIVE_TO_CUBE_JOYSTICK_BUTTON_NUMBER).whenPressed(new DriveToCubeJoystickCommand());
		new JoystickButton(driveLeft, DRIVE_TO_CUBE_JOYSTICK_BUTTON_NUMBER).whenReleased(new DoNothingCommand(Drive.getInstance()));
	
		DriveTuningCommand tuningCommand = new DriveTuningCommand();
		new JoystickButton(driveLeft, DRIVE_TUNING_BUTTON_NUMBER).whenPressed(tuningCommand);
		
		new JoystickButton(driveLeft, TURN_TO_ANGLE_COMMAND_BUTTON).whenPressed(new TurnToAngleCommand(new Angle(180, Angle.Unit.DEGREE)));
		
	}

	public void initDriveRight() {

		new JoystickButton(driveRight, SNIPER_MODE_BUTTON).whenPressed(new EnableSniperTurnMode(true));
		new JoystickButton(driveRight, SNIPER_MODE_BUTTON).whenReleased(new EnableSniperTurnMode(false));
		
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
			
		new JoystickButton(driveRight,
				 TEST_SEQUENCE_BUTTON_NUMBER).whenPressed(new TestSequence());
	}
}
