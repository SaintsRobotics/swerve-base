// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AllianceFlipUtil;

public class DriveToPose extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private Pose2d currentPose;
  private final Pose2d m_targetPose;
  private Timer m_timer = new Timer();

  private final TrapezoidProfile.Constraints constraints = new Constraints(DriveConstants.kMaxSpeedMetersPerSecond / 3, 5 / 2);
  private final TrapezoidProfile.Constraints angularConstraints = new Constraints(
      DriveConstants.kMaxAngularSpeedRadiansPerSecond / 10, 5 / 10);

  private final ProfiledPIDController xController = new ProfiledPIDController(3.0, 0.01, 0, constraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(3.0, 0.01, 0, constraints);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(8.0, 0.0, 0.0, angularConstraints);

  /** Creates a new DriveToPose. */
  public DriveToPose(DriveSubsystem driveSubsystem, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    xController.setTolerance(0.005, 0.05);
    yController.setTolerance(0.005, 0.05);
    thetaController.setTolerance(Math.toRadians(1.5), 0.05);

    thetaController.enableContinuousInput(0, Math.PI * 2);

    m_driveSubsystem = driveSubsystem;
    m_targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();

    // TODO: reset state with current chassis speeds
    currentPose = m_driveSubsystem.getPose();
    xController.reset(new State(currentPose.getX(), 0));
    yController.reset(new State(currentPose.getY(), 0));
    thetaController.reset(new State(currentPose.getRotation().getRadians(), 0));

    SmartDashboard.putNumber("drive target X", m_targetPose.getX());
    SmartDashboard.putNumber("drive target Y", m_targetPose.getY());
    SmartDashboard.putNumber("drive target Rot", m_targetPose.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = m_driveSubsystem.getPose();

    double xSpeed = xController.calculate(currentPose.getX(), m_targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), m_targetPose.getY());
    double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(),
        m_targetPose.getRotation().getRadians());

    if (AllianceFlipUtil.shouldFlip()){
        xSpeed = -xSpeed;
        ySpeed = -ySpeed;
    }

    if (DriveConstants.kAutoDriving) {
      m_driveSubsystem.drive(xSpeed, ySpeed, thetaSpeed, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !DriveConstants.kAutoDriving || (xController.atGoal() &&
        yController.atGoal() &&
        thetaController.atGoal());
  }
}
