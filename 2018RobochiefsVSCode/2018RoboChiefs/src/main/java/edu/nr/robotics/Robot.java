
package edu.nr.robotics;

import edu.nr.lib.commandbased.CancelAllCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.gyro.ResetGyroCommand;
import edu.nr.lib.interfaces.Periodic;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.lib.network.LimelightNetworkTable;
import edu.nr.lib.units.Distance;
import edu.nr.lib.units.Time;
import edu.nr.lib.units.Speed;
import edu.nr.robotics.FieldData.Direction;
import edu.nr.robotics.auton.AutoChoosers;
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
import edu.nr.robotics.auton.automap.StartPosRightSwitchRightCommand;
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
import edu.nr.robotics.subsystems.drive.Drive;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.nr.robotics.subsystems.elevator.Elevator;
import edu.nr.robotics.subsystems.elevator.ElevatorDeltaPositionSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorMoveBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorPositionSmartDashboardCommand;
import edu.nr.robotics.subsystems.elevator.ElevatorProfileSmartDashboardCommandGroup;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooter;
import edu.nr.robotics.subsystems.elevatorShooter.ElevatorShooterVelocitySmartDashboardCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorDeltaPositionSmartDashboardCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorMoveBasicSmartDashboardCommand;
import edu.nr.robotics.subsystems.intakeElevator.IntakeElevatorProfileSmartDashboardCommandGroup;
import edu.nr.robotics.subsystems.intakeRollers.IntakeRollersVelocitySmartDashboardCommand;

import edu.nr.robotics.subsystems.intakeRollers.IntakeRollers;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
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
public class Robot extends TimedRobot {
	
	private double prevTime = 0;
	private double dt = 0;
	private double dtAv = 0;
	private int count = 0;
	
	private static Robot singleton;
	
	public synchronized static Robot getInstance() {
		return singleton;
	}

	private Command autonomousCommand;
	public AutoChoosers.StartPos selectedStartPos;
	public AutoChoosers.Switch selectedSwitch;
	public AutoChoosers.Scale selectedScale;
	public AutoChoosers.AllianceBlocks selectedBlocks;
	public AutoChoosers.ScaleSwitch selectedScaleSwitch;
	public double autoWaitTime;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		singleton = this;

		m_period = 0.010;
		
		smartDashboardInit();
		autoChooserInit();
		OI.init();
		LimelightNetworkTable.getInstance().lightLED(true);
		LimelightNetworkTable.getInstance().lightLED(false);
		
		//CameraInit();

		for (int i = 0; i < 10; i++) {
			System.out.println("Name Geoff");
		}
		System.out.println("-Erik");
	}

	public void CameraInit() {
		new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(720, 1080);
			
		}).start();
		
	}

	public void autoChooserInit() {
		
		AutoChoosers.autoStartPosChooser.addDefault("Start Pos Left", StartPos.left);
		AutoChoosers.autoStartPosChooser.addObject("Start Pos Middle", StartPos.middle);
		AutoChoosers.autoStartPosChooser.addObject("Start Pos Right", StartPos.right);
		
		AutoChoosers.autoSwitchChooser.addDefault("Switch None", Switch.other);
		AutoChoosers.autoSwitchChooser.addObject("Switch Left", Switch.leftOnly);
		AutoChoosers.autoSwitchChooser.addObject("Switch Right", Switch.rightOnly);
		AutoChoosers.autoSwitchChooser.addObject("Switch Both", Switch.both);
		
		AutoChoosers.autoScaleChooser.addDefault("Scale Default", Scale.none);
		AutoChoosers.autoScaleChooser.addObject("Scale Left", Scale.leftonly);
		AutoChoosers.autoScaleChooser.addObject("Scale Right", Scale.rightonly);
		AutoChoosers.autoScaleChooser.addObject("Scale Both", Scale.both);
		
		AutoChoosers.allianceBlockChooser.addDefault("None", AllianceBlocks.none);
		AutoChoosers.allianceBlockChooser.addObject("Block 1", AllianceBlocks.block1);
		AutoChoosers.allianceBlockChooser.addObject("Block 6", AllianceBlocks.block6);
		AutoChoosers.allianceBlockChooser.addObject("Both", AllianceBlocks.both);
		
		AutoChoosers.autoScaleSwitchChooser.addObject("Scale Switch Yes", ScaleSwitch.scaleSwitchYes);
		AutoChoosers.autoScaleSwitchChooser.addDefault("Scale Switch No", ScaleSwitch.scaleSwitchNo);
		
		SmartDashboard.putData("Auto Start Pos", AutoChoosers.autoStartPosChooser);
		SmartDashboard.putData("Auto Switch", AutoChoosers.autoSwitchChooser);
		SmartDashboard.putData("Auto Scale", AutoChoosers.autoScaleChooser);
		SmartDashboard.putData("Partner Blocks", AutoChoosers.allianceBlockChooser);
		SmartDashboard.putData("Auto ScaleSwitch", AutoChoosers.autoScaleSwitchChooser);
	}
	//a
	/**
	 * What SmartDashboard puts on initialization (usually commands)
	 */
	public void smartDashboardInit() {
		SmartDashboard.putData(new CSVSaverEnable());
		SmartDashboard.putData(new CSVSaverDisable());
		SmartDashboard.putData(new TestShootingWhileTurningSmartDashboardCommand());
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
			SmartDashboard.putData(new ClimbButtonCommand());
		}		
		
		//Elevator
		if(EnabledSubsystems.ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putData(new ElevatorDeltaPositionSmartDashboardCommand());
			SmartDashboard.putData(new ElevatorMoveBasicSmartDashboardCommand());	
			SmartDashboard.putData(new ElevatorProfileSmartDashboardCommandGroup());
			SmartDashboard.putData(new ElevatorPositionSmartDashboardCommand());
		}
		
		//Elevator Shooter
		if(EnabledSubsystems.ELEVATOR_SHOOTER_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putData(new ElevatorShooterVelocitySmartDashboardCommand());	
		}
		
		//Intake Elevator
		if(EnabledSubsystems.INTAKE_ELEVATOR_SMARTDASHBOARD_DEBUG_ENABLED) {
			SmartDashboard.putData(new IntakeElevatorDeltaPositionSmartDashboardCommand());
			SmartDashboard.putData(new IntakeElevatorMoveBasicSmartDashboardCommand());	
			SmartDashboard.putData(new IntakeElevatorProfileSmartDashboardCommandGroup());
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
	public void testInit() {
		
		if(EnabledSubsystems.DRIVE_ENABLED ){
			SmartDashboard.putNumber("Left Drive Percent: ", 0);
			SmartDashboard.putNumber("Right Drive Percent: ", 0);
			SmartDashboard.putNumber("H Drive Percent: ", 0);

			SmartDashboard.putNumber("Right Drive Current: ", Drive.getInstance().getRightCurrent());
			SmartDashboard.putNumber("Left Drive Current: ", Drive.getInstance().getLeftCurrent());
			SmartDashboard.putNumber("H Drive Current: ", Drive.getInstance().getHCurrent());
		}

		if(EnabledSubsystems.ELEVATOR_ENABLED) {
			SmartDashboard.putNumber("Elevator Percent: ", 0);
			SmartDashboard.putNumber("Elevator Current: ", Elevator.getInstance().getMasterCurrent());
			SmartDashboard.putNumber("Elevator Follow Current; ", Elevator.getInstance().getFollowerCurrent());

			SmartDashboard.putNumber("Elevator Position: ", 0);
			SmartDashboard.putNumber("Elevator Velocity: ", Elevator.getInstance().getVelocity().get(Distance.Unit.FOOT, Time.Unit.SECOND));
		}

		if(EnabledSubsystems.INTAKE_ROLLERS_ENABLED) {
			SmartDashboard.putNumber("Right Intake Current: ", IntakeRollers.getInstance().getCurrentRight());
			SmartDashboard.putNumber("Left Intake Current: ", IntakeRollers.getInstance().getCurrentLeft());

			SmartDashboard.putNumber("Left Intake Percent: ", 0); // should get actual percent, this is a test.
			SmartDashboard.putNumber("Right Intake Percent: ", 0);

		}

		if(EnabledSubsystems.ELEVATOR_SHOOTER_ENABLED) {
			SmartDashboard.putNumber("Elevator Shooter Percent: ", 0);
			SmartDashboard.putNumber("Elevator Shooter Current: ", 0);
		}
	
	//	SmartDashboard.putNumber("Intake Elevator Position: ", 0);
	//	SmartDashboard.putNumber("Intake Elevator Current: ", 0);

	}

	@Override
	public void disabledPeriodic() {
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

		/*SmartDashboard.putBoolean("Near Switch Left: ",
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
								&& FieldData.getInstance().alliance == Alliance.Blue));*/

		selectedStartPos = AutoChoosers.autoStartPosChooser.getSelected();
		selectedSwitch = AutoChoosers.autoSwitchChooser.getSelected();
		selectedScale = AutoChoosers.autoScaleChooser.getSelected();
		selectedBlocks = AutoChoosers.allianceBlockChooser.getSelected();
		selectedScaleSwitch = AutoChoosers.autoScaleSwitchChooser.getSelected();
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
	}

	@Override
	public void teleopInit() {
		
		new CancelAllCommand().start();
		
		new ClimberCoastCommand(false).start();
			
		LimelightNetworkTable.getInstance().lightLED(true);
		LimelightNetworkTable.getInstance().lightLED(false);
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		count++;
		dt = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - prevTime;
		prevTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		dtAv += dt;

		if (count % 100 == 0) {
			//System.out.println(dtAv / 100.0);
			dtAv = 0;
			count = 0;
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
// TEST MODE IS GOOD, not elevator though
		Drive.getInstance().setMotorSpeedRaw(SmartDashboard.getNumber("Left Drive Percent: ", 0),SmartDashboard.getNumber("Right Drive Percent: ", 0),SmartDashboard.getNumber("H Drive Percent: ", 0));
	
	//	System.out.println(SmartDashboard.getNumber("Elevator Position: ", 0));

		Elevator.getInstance().setPosition(new Distance(SmartDashboard.getNumber("Elevator Position: ", 0), Distance.Unit.FOOT));

		
		ElevatorShooter.getInstance().setMotorSpeedPercent(SmartDashboard.getNumber("Elevator Shooter Percent: ", 0));

		IntakeRollers.getInstance().setMotorSpeedPercent(SmartDashboard.getNumber("Left Intake Percent: ", 0), SmartDashboard.getNumber("Right Intake Percent: ", 0));
	//	IntakeElevator.getInstance().setPosition(SmartDashboard.getNumber("Intake Elevator Position: ", IntakeElevator.getInstance().))


	}
	
	@Override
	public void robotPeriodic() {
		Scheduler.getInstance().run();
		
		Periodic.runAll();
		SmartDashboardSource.runAll();
		
	}
	
	public Command getAutoCommand() {
		if (selectedScaleSwitch == AutoChoosers.ScaleSwitch.scaleSwitchYes
				&& (((selectedScale == AutoChoosers.Scale.leftonly || selectedScale == AutoChoosers.Scale.both)
						&& FieldData.getInstance().scale == Direction.left)
						|| ((selectedScale == AutoChoosers.Scale.rightonly || selectedScale == AutoChoosers.Scale.both)
								&& FieldData.getInstance().scale == Direction.right))) {
			if(selectedStartPos == AutoChoosers.StartPos.left && FieldData.getInstance().scale == Direction.left) {
				return (new StartPosLeftScaleSwitchLeftCommand());
			}
			else if(selectedStartPos == AutoChoosers.StartPos.left && FieldData.getInstance().scale == Direction.right) {
				return (new StartPosLeftScaleSwitchRightCommand());
			}
			else if(selectedStartPos == AutoChoosers.StartPos.right && FieldData.getInstance().scale == Direction.left) {
				return (new StartPosRightScaleSwitchLeftCommand());
			}
			else if(selectedStartPos == AutoChoosers.StartPos.right && FieldData.getInstance().scale == Direction.right) {
				return (new StartPosRightScaleSwitchRightCommand());
			} return (new DriveOverBaselineAutoCommand());
		}
		else {
			if (selectedStartPos == AutoChoosers.StartPos.left) {
				if (selectedSwitch == AutoChoosers.Switch.other || selectedSwitch == AutoChoosers.Switch.rightOnly) {
					return (new StartPosLeftSwitchOtherCommand());
				} else if (selectedSwitch == AutoChoosers.Switch.leftOnly || selectedSwitch == AutoChoosers.Switch.both) {
					return new StartPosLeftSwitchLeftCommand();			
				}
			} else if (selectedStartPos == AutoChoosers.StartPos.middle) {
				if (selectedSwitch == AutoChoosers.Switch.other) {
					return (new StartPosMiddleSwitchOtherCommand());		
				} else if (selectedSwitch == AutoChoosers.Switch.leftOnly) {
					return (new StartPosMiddleSwitchLeftCommand());		
				} else if (selectedSwitch == AutoChoosers.Switch.rightOnly){
					return (new StartPosMiddleSwitchRightCommand());
				} else if (selectedSwitch == AutoChoosers.Switch.both) {
					return (new StartPosMiddleSwitchBothCommand());
				}
			} else if (selectedStartPos == AutoChoosers.StartPos.right) {
				if (selectedSwitch == AutoChoosers.Switch.other || selectedSwitch == AutoChoosers.Switch.leftOnly) {
					return (new StartPosRightSwitchOtherCommand());
				} else if (selectedSwitch == AutoChoosers.Switch.rightOnly || selectedSwitch == AutoChoosers.Switch.both){
					return (new StartPosRightSwitchRightCommand());
				}
			} return new DriveOverBaselineAutoCommand();
		}
	}
}
