
package edu.nr.robotics;

import edu.nr.lib.commandbased.CancelAllCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.ResetGyroCommand;
import edu.nr.lib.interfaces.Periodic;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.AutoChoosers.AllianceBlocks;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.AutoChoosers.StartPos;
import edu.nr.robotics.auton.AutoChoosers.Switch;
import edu.nr.robotics.auton.DriveOverBaselineAutoCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchNoneCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchRightCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchBothCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchNoneCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchBothCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchNoneCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchRightCommand;
import edu.nr.robotics.subsystems.EnabledSubsystems;
import edu.nr.robotics.subsystems.climber.ClimberCoastCommand;
import edu.nr.robotics.subsystems.climber.ClimberCurrentSmartDashboardCommand;
import edu.nr.robotics.subsystems.cubeHandler.CubeHandlerVelocitySmartDashboardCommand;
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
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersVelocitySmartDashboardCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	private static Robot singleton;
	
	public synchronized static Robot getInstance() {
		return singleton;
	}

	private Command autonomousCommand;
	public AutoChoosers.StartPos selectedStartPos;
	public AutoChoosers.Switch selectedSwitch;
	public AutoChoosers.Scale selectedScale;
	public AutoChoosers.AllianceBlocks selectedBlocks;
	public double autoWaitTime;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		singleton = this;
		
		smartDashboardInit();
		autoChooserInit();
		OI.init();
		LimelightNetworkTable.getInstance();
		LimelightNetworkTable.getInstance().lightLED(false);
	}

	public void autoChooserInit() {
		
		AutoChoosers.autoStartPosChooser.addDefault("Start Pos Left", StartPos.left);
		AutoChoosers.autoStartPosChooser.addObject("Start Pos Middle", StartPos.middle);
		AutoChoosers.autoStartPosChooser.addObject("Start Pos Right", StartPos.right);
		
		AutoChoosers.autoSwitchChooser.addDefault("Switch None", Switch.none);
		AutoChoosers.autoSwitchChooser.addObject("Switch Left Only", Switch.leftOnly);
		AutoChoosers.autoSwitchChooser.addObject("Switch Right Only", Switch.rightOnly);
		AutoChoosers.autoSwitchChooser.addObject("Switch Both", Switch.both);
		
		AutoChoosers.autoScaleChooser.addDefault("Scale None", Scale.none);
		AutoChoosers.autoScaleChooser.addObject("Scale Left Only", Scale.leftonly);
		AutoChoosers.autoScaleChooser.addObject("Scale Right Only", Scale.rightonly);
		AutoChoosers.autoScaleChooser.addObject("Scale Both", Scale.both);
		
		AutoChoosers.allianceBlockChooser.addDefault("None", AllianceBlocks.none);
		AutoChoosers.allianceBlockChooser.addObject("Block 1", AllianceBlocks.block1);
		AutoChoosers.allianceBlockChooser.addObject("Block 6", AllianceBlocks.block6);
		AutoChoosers.allianceBlockChooser.addObject("Both", AllianceBlocks.both);
		
		SmartDashboard.putData("Auto StartPosition", AutoChoosers.autoStartPosChooser);
		SmartDashboard.putData("Auto Switch", AutoChoosers.autoSwitchChooser);
		SmartDashboard.putData("Auto Scale", AutoChoosers.autoScaleChooser);
		SmartDashboard.putData("Alliance Partner Blocks", AutoChoosers.allianceBlockChooser);
	}

	/**
	 * What SmartDashboard puts on initialization (usually commands)
	 */
	public void smartDashboardInit() {
		SmartDashboard.putData(new CSVSaverEnable());
		SmartDashboard.putData(new CSVSaverDisable());
		SmartDashboard.putNumber("Auto Wait Time", 0);
		
		//Drive
		if(EnabledSubsystems.DRIVE_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putData(new DriveForwardBasicSmartDashboardCommand());
			SmartDashboard.putData(new EnableMotionProfileSmartDashboardCommand());
			SmartDashboard.putData(new DriveForwardSmartDashboardCommandH());
			SmartDashboard.putData(new TurnSmartDashboardCommand());
		}
		
		//Climber
		if(EnabledSubsystems.CLIMBER_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putData(new ClimberCurrentSmartDashboardCommand());
		}
		
		//Cube Handler
		if(EnabledSubsystems.CUBE_HANDLER_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putData(new CubeHandlerVelocitySmartDashboardCommand());	
		}
		
		
		//Elevator
		if(EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putData(new ElevatorDeltaPositionSmartDashboardCommand());
			SmartDashboard.putData(new ElevatorMoveBasicSmartDashboardCommand());	
			SmartDashboard.putData(new ElevatorProfileSmartDashboardCommandGroup());
		}
		
		//Elevator Shooter
		if(EnabledSubsystems.ELEVATOR_SHOOTER_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putData(new ElevatorShooterVelocitySmartDashboardCommand());	
		}
		
		//Intake Elevator
		if(EnabledSubsystems.INTAKE_ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putData(new IntakeElevatorDeltaPositionSmartDashboardCommand());
			SmartDashboard.putData(new IntakeElevatorMoveBasicSmartDashboardCommand());	
		}
		
		//Intake Rollers
		if(EnabledSubsystems.INTAKE_ROLLERS_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putData(new IntakeRollersVelocitySmartDashboardCommand());
		}
	}
	
	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		for (NRSubsystem subsystem : NRSubsystem.subsystems) {
			subsystem.disable();
		}
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		
		LimelightNetworkTable.getInstance().lightLED(false);
		new ResetGyroCommand().start();
		new ClimberCoastCommand(false).start();
		
		FieldData.getInstance().getFieldData();

		SmartDashboard.putBoolean("Near Switch Left: ",
				(FieldData.getInstance().nearSwitch == Direction.left
						&& FieldData.getInstance().alliance == Alliance.Blue)
						|| (FieldData.getInstance().nearSwitch == Direction.right
								&& FieldData.getInstance().alliance == Alliance.Red));
		SmartDashboard.putBoolean("Near Switch Right: ",
				(FieldData.getInstance().nearSwitch == Direction.left
						&& FieldData.getInstance().alliance == Alliance.Red)
						|| (FieldData.getInstance().nearSwitch == Direction.right
								&& FieldData.getInstance().alliance == Alliance.Blue));
		SmartDashboard.putBoolean("Scale Left: ",
				(FieldData.getInstance().scale == Direction.left
						&& FieldData.getInstance().alliance == Alliance.Blue)
						|| (FieldData.getInstance().scale == Direction.right
								&& FieldData.getInstance().alliance == Alliance.Red));
		SmartDashboard.putBoolean("Scale Right: ",
				(FieldData.getInstance().scale == Direction.left
						&& FieldData.getInstance().alliance == Alliance.Red)
						|| (FieldData.getInstance().scale == Direction.right
								&& FieldData.getInstance().alliance == Alliance.Blue));
		SmartDashboard.putBoolean("Far Switch Left: ",
				(FieldData.getInstance().farSwitch == Direction.left
						&& FieldData.getInstance().alliance == Alliance.Blue)
						|| (FieldData.getInstance().farSwitch == Direction.right
								&& FieldData.getInstance().alliance == Alliance.Red));
		SmartDashboard.putBoolean("Far Switch Right: ",
				(FieldData.getInstance().farSwitch == Direction.left
						&& FieldData.getInstance().alliance == Alliance.Red)
						|| (FieldData.getInstance().farSwitch == Direction.right
								&& FieldData.getInstance().alliance == Alliance.Blue));

		selectedStartPos = AutoChoosers.autoStartPosChooser.getSelected();
		selectedSwitch = AutoChoosers.autoSwitchChooser.getSelected();
		selectedScale = AutoChoosers.autoScaleChooser.getSelected();
		selectedBlocks = AutoChoosers.allianceBlockChooser.getSelected();
		autoWaitTime = SmartDashboard.getNumber("Auto Wait Time", autoWaitTime);
		
		
		autonomousCommand = getAutoCommand();

		System.out.println("Start Pos: " + selectedStartPos + " Switch: " + selectedSwitch + " Scale: " + selectedScale);
		
		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		
		new CancelAllCommand().start();
		
		new ClimberCoastCommand(false).start();
		
		LimelightNetworkTable.getInstance().lightLED(false);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void robotPeriodic() {
		Scheduler.getInstance().run();
		
		Periodic.runAll();
		SmartDashboardSource.runAll();
		
		SmartDashboard.putData(RobotDiagram.getInstance());
	}
	
	public Command getAutoCommand() {
		if (selectedStartPos == AutoChoosers.StartPos.left) {
			if (selectedSwitch == AutoChoosers.Switch.none || selectedSwitch == AutoChoosers.Switch.rightOnly) {
				return (new StartPosLeftSwitchNoneCommand());
			} else if (selectedSwitch == AutoChoosers.Switch.leftOnly) {
				return new StartPosLeftSwitchLeftCommand();			
			} else if (selectedSwitch == AutoChoosers.Switch.both) {
				return new StartPosLeftSwitchBothCommand();		
			}
		} else if (selectedStartPos == AutoChoosers.StartPos.middle) {
			if (selectedSwitch == AutoChoosers.Switch.none) {
				return (new StartPosMiddleSwitchNoneCommand());		
			} else if (selectedSwitch == AutoChoosers.Switch.leftOnly) {
				return (new StartPosMiddleSwitchLeftCommand());		
			} else if (selectedSwitch == AutoChoosers.Switch.rightOnly){
				return (new StartPosMiddleSwitchRightCommand());
			} else if (selectedSwitch == AutoChoosers.Switch.both) {
				return (new StartPosMiddleSwitchBothCommand());
			}
		} else if (selectedStartPos == AutoChoosers.StartPos.right) {
			if (selectedSwitch == AutoChoosers.Switch.none || selectedSwitch == AutoChoosers.Switch.leftOnly) {
				return (new StartPosRightSwitchNoneCommand());
			} else if (selectedSwitch == AutoChoosers.Switch.rightOnly || selectedSwitch == AutoChoosers.Switch.both){
				return (new StartPosRightSwitchRightCommand());
			}
		} return new DriveOverBaselineAutoCommand();
	}
}
