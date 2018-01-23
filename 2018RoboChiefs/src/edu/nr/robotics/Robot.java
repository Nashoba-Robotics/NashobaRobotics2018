
package edu.nr.robotics;

import edu.nr.lib.commandbased.CancelAllCommand;
import edu.nr.lib.commandbased.DoNothingCommand;
import edu.nr.lib.commandbased.NRSubsystem;
import edu.nr.lib.interfaces.Periodic;
import edu.nr.lib.interfaces.SmartDashboardSource;
import edu.nr.robotics.auton.AutoChoosers;
import edu.nr.robotics.auton.AutoChoosers.AllianceBlocks;
import edu.nr.robotics.auton.AutoChoosers.Scale;
import edu.nr.robotics.auton.AutoChoosers.StartPos;
import edu.nr.robotics.auton.AutoChoosers.Switch;
import edu.nr.robotics.auton.automap.StartPosFarRightSwitchNoneCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchBothCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosLeftSwitchNoneCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchBothCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchLeftCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchNoneCommand;
import edu.nr.robotics.auton.automap.StartPosMiddleSwitchRightCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchBothCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchNoneCommand;
import edu.nr.robotics.auton.automap.StartPosRightSwitchRightCommand;
import edu.nr.robotics.subsystems.drive.CSVSaverDisable;
import edu.nr.robotics.subsystems.drive.CSVSaverEnable;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.WaitCommand;
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
	}

	public void autoChooserInit() {
		
		AutoChoosers.autoStartPosChooser.addDefault("Left", StartPos.left);
		AutoChoosers.autoStartPosChooser.addObject("Middle", StartPos.middle);
		AutoChoosers.autoStartPosChooser.addObject("Right", StartPos.right);
		AutoChoosers.autoStartPosChooser.addObject("Far Right", StartPos.farRight);
		
		AutoChoosers.autoSwitchChooser.addDefault("Left Only", Switch.leftOnly);
		AutoChoosers.autoSwitchChooser.addObject("RightOnly", Switch.rightOnly);
		AutoChoosers.autoSwitchChooser.addObject("None", Switch.none);
		AutoChoosers.autoSwitchChooser.addObject("both", Switch.both);
		
		AutoChoosers.autoScaleChooser.addDefault("Scale", Scale.yes);
		AutoChoosers.autoScaleChooser.addObject("No Scale", Scale.no);
		
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
		FieldData.getInstance().getFieldData();
		
		selectedStartPos = AutoChoosers.autoStartPosChooser.getSelected();
		selectedSwitch = AutoChoosers.autoSwitchChooser.getSelected();
		selectedScale = AutoChoosers.autoScaleChooser.getSelected();
		selectedBlocks = AutoChoosers.allianceBlockChooser.getSelected();
		autoWaitTime = SmartDashboard.getNumber("Auto Wait Time", autoWaitTime);
		
		
		autonomousCommand = getAutoCommand();

		System.out.println("Initializing auto command: " + autonomousCommand);
		
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
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
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
			if (selectedSwitch == AutoChoosers.Switch.none) {
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
			if (selectedSwitch == AutoChoosers.Switch.none) {
				return (new StartPosRightSwitchNoneCommand());	
			} else if (selectedSwitch == AutoChoosers.Switch.rightOnly){
				return (new StartPosRightSwitchRightCommand());
			} else if (selectedSwitch == AutoChoosers.Switch.both) {
				return (new StartPosRightSwitchBothCommand());
			}
		} else if (selectedStartPos == AutoChoosers.StartPos.farRight) {
			if (selectedSwitch == AutoChoosers.Switch.none) {
				return (new StartPosFarRightSwitchNoneCommand());
			} else if (selectedSwitch == AutoChoosers.Switch.rightOnly){
				return (new StartPosRightSwitchRightCommand());
			}
		} return new DoNothingCommand();
	}
}
