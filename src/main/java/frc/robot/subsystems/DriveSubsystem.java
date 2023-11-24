// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftDriveMotorReversed,
      DriveConstants.kFrontLeftTurningEncoderOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kRearLeftDriveMotorReversed,
      DriveConstants.kRearLeftTurningEncoderOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightDriveMotorReversed,
      DriveConstants.kFrontRightTurningEncoderOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kRearRightDriveMotorReversed,
      DriveConstants.kRearRightTurningEncoderOffset);

  private final AHRS m_gyro = new AHRS();
  private double m_gyroAngle;

  private final Timer m_headingCorrectionTimer = new Timer();
  private final PIDController m_headingCorrectionPID = new PIDController(DriveConstants.kPHeadingCorrectionController, 0, 0);
  private double m_desiredHeading = 0;

  private SwerveModulePosition[] m_swerveModulePositions = new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
  };

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(), m_swerveModulePositions);

  private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putData("Field", m_field);
    m_headingCorrectionTimer.restart();
    m_headingCorrectionPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_swerveModulePositions = new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };

    m_odometry.update(Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle), m_swerveModulePositions);

    m_field.setRobotPose(m_odometry.getPoseMeters());

    SmartDashboard.putNumber("gyro angle", m_gyro.getAngle());
    SmartDashboard.putNumber("odometryX", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometryY", m_odometry.getPoseMeters().getY());

    // AdvantageScope Logging
    double[] logData = {
        m_frontLeft.getPosition().angle.getDegrees(), m_frontLeft.driveOutput,
        m_frontRight.getPosition().angle.getDegrees(), m_frontRight.driveOutput,
        m_rearLeft.getPosition().angle.getDegrees(), m_rearLeft.driveOutput,
        m_rearRight.getPosition().angle.getDegrees(), m_rearRight.driveOutput,
    };
    SmartDashboard.putNumberArray("AdvantageScope Swerve States", logData);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotation      Angular rotation speed of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
    // Check if current heading should be considered desired heading
    if (rotation == 0 || m_headingCorrectionTimer.hasElapsed(DriveConstants.kHeadingCorrectionTurningStopTime)) {
      m_desiredHeading = getPose().getRotation().getRadians();
      m_headingCorrectionTimer.restart();
    }

    double calculatedRotation = rotation;

    // Calculate rotation speed using PID if zero rotation input
    if (rotation == 0) {
      calculatedRotation = m_headingCorrectionPID.calculate(getPose().getRotation().getRadians(), m_desiredHeading);
    }

    // Depending on whether the robot is being driven in field relative, calculate
    // the desired states for each of the modules
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, calculatedRotation,
                Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle))
            : new ChassisSpeeds(xSpeed, ySpeed, calculatedRotation));

    setModuleStates(swerveModuleStates);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_gyroAngle = 0;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);

    // AdvantageScope Logging
    double[] logData = {
        desiredStates[0].angle.getDegrees(), desiredStates[0].speedMetersPerSecond,
        desiredStates[1].angle.getDegrees(), desiredStates[1].speedMetersPerSecond,
        desiredStates[2].angle.getDegrees(), desiredStates[2].speedMetersPerSecond,
        desiredStates[3].angle.getDegrees(), desiredStates[3].speedMetersPerSecond,
    };
    SmartDashboard.putNumberArray("AdvantageScope Swerve Desired States", logData);

    // Takes the integral of the rotation speed to find the current angle for the
    // simulator
    m_gyroAngle += DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates).omegaRadiansPerSecond
        * Robot.kDefaultPeriod;
  }

}
