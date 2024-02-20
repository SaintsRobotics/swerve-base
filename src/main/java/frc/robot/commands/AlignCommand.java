// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private double m_posex = 0;
  private double m_posey = 0;
  private double m_rot = 0;


  /** Creates a new AlignCommand. */
  public AlignCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, DriveSubsystem subsystem) {
      m_driveSubsystem = subsystem;

      m_posex = x.getAsDouble();
      m_posey = y.getAsDouble();
      m_rot = rot.getAsDouble();
  }

  @Override
  public void initialize() {} 

  @Override
  public void execute() {
      AutoBuilder.pathfindToPose(new Pose2d(m_posex, m_posey, new Rotation2d(m_rot)), new PathConstraints(
             DriveConstants.kMaxSpeedMetersPerSecond - 1, 5, DriveConstants.kMaxAngularSpeedRadiansPerSecond - 1, 5));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
