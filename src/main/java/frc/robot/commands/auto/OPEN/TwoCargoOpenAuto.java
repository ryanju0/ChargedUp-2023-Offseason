// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.OPEN;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.auto.AutoBase;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Superstructure.BUTTON;
import frc.robot.subsystems.Superstructure.DPAD;
import frc.robot.subsystems.Superstructure.ScoreMode;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class TwoCargoOpenAuto extends AutoBase {
	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"2CargoOPEN",
			new PathConstraints(1.8, 3.0));

	private final HashMap<String, Command> AutoEventMap = new HashMap<>();

	public TwoCargoOpenAuto(DriveSubsystem m_robotDrive, Superstructure m_superstructure, Shooter m_shooter) {
		super(m_robotDrive);

		AutoEventMap.put("intake cube", 
			new SequentialCommandGroup(
				m_superstructure.setScoreLevelCommand(BUTTON.X),
				m_superstructure.setSubsystemState(DPAD.LEFT)));
		AutoEventMap.put("set shooter high", 
			new SequentialCommandGroup(
				m_superstructure.setScoreLevelCommand(BUTTON.Y),
				m_superstructure.setSubsystemState(DPAD.RIGHT)));
		AutoEventMap.put("shoot cube", 
			new SequentialCommandGroup(
				m_shooter.setKickerOuttakeCommand(ShooterConstants.kKickSpeed),
				new WaitCommand(0.2),
				m_shooter.stopCommand()));
		AutoEventMap.put("retract shooter", 
			m_superstructure.setSubsystemState(DPAD.DOWN));

		SwerveAutoBuilder autoBuilder = getSwerveAutoBuilder(AutoEventMap);

		addCommands(
            //Score preload
			m_superstructure.scorePreloadedCone(3.4),

      		m_superstructure.setScoreModeCommand(ScoreMode.SHOOTER),

			//Follow Path
			autoBuilder.fullAuto(autoPathGroup)
		);
	}
}