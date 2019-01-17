package edu.nr.robotics;

import edu.nr.lib.commandbased.CancelAllCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.ResetGyroCommand;
import edu.nr.lib.interfaces.Periodic;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.robotics.FieldData;
import edu.nr.robotics.OI;
import edu.nr.robotics.Robot;
import edu.nr.robotics.FieldData.Direction;
/*import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.AutoChoosers.AllianceBlocks;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.AutoChoosers.ScaleSwitch;
import edu.nr.robotics.auton.AutoChoosers.StartPos;
import edu.nr.robotics.auton.AutoChoosers.Switch;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.automap.StartPosLeftScaleSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosLeftScaleSwitchRightCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchOtherCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchBothCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchOtherCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchRightCommand;
import edu.nr.robotics.auton.automap.StartPosRightScaleSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosRightScaleSwitchRightCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchOtherCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchRightCommand;*/
import edu.nr.robotics.multicommands.ClimbButtonCommand;
import edu.nr.robotics.multicommands.TestShootingWhileTurningSmartDashboardCommand;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.climber.ClimberCoastCommand;
import edu.nr.robotics.subsystems.drive.CSVSaverDisable;
import edu.nr.robotics.subsystems.drive.CSVSaverEnable;
import edu.nr.robotics.subsystems.drive.DriveForwardBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.DriveForwardSmartDashboardCommandH;
import edu.nr.robotics.subsystems.drive.EnableMotionProfileSmartDashboardCommand;
import edu.nr.robotics.subsystems.drive.TurnSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorDeltaPositionSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorMoveBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileSmartDashboardCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterVelocitySmartDashboardCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorDeltaPositionSmartDashboardCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorMoveBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorProfileSmartDashboardCommandGroup;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersVelocitySmartDashboardCommand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class RobotMap {
	public static final int LEFT_DRIVE_FOLLOW = 13;
	public static final int RIGHT_DRIVE_FOLLOW = 2;
	public static final int H_DRIVE_FOLLOW = 0;
	public static final int RIGHT_DRIVE = 3;
	public static final int LEFT_DRIVE = 12;
	public static final int H_DRIVE = 1;
	public static final int ELEVATOR_TALON = 15;
	public static final int ELEVATOR_TALON_FOLLOW = 14;
	public static final int INTAKE_ELEVATOR_TALON = 5;
	public static final int ELEVATOR_SHOOTER_TALON = 4;
	public static final int INTAKE_ROLLERS_LEFT = 10;
	public static final int INTAKE_ROLLERS_RIGHT = 6;
	public static final int CLIMBER_TALON = 11;
	
	public static final int INTAKE_SENSOR_PORT_LEFT = 7;
	public static final int INTAKE_SENSOR_PORT_RIGHT = 9;
	public static final int ELEVATOR_SENSOR_PORT = 3;
	public static final int FLOOR_SENSOR_PORT = 8;
	
}

