// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftDriveMotorReversed,
      DriveConstants.kFrontLeftTurningEncoderOffset);

  private final SwerveModule rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kRearLeftDriveMotorReversed,
      DriveConstants.kRearLeftTurningEncoderOffset);

  private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightDriveMotorReversed,
      DriveConstants.kFrontRightTurningEncoderOffset);

  private final SwerveModule rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kRearRightDriveMotorReversed,
      DriveConstants.kRearRightTurningEncoderOffset);

  private final AHRS gyro = new AHRS();
  private double GYRO_ANGLE;
  private SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      rearLeft.getPosition(),
      rearRight.getPosition()
  };
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
      gyro.getRotation2d(), swerveModulePositions, new Pose2d(), VisionConstants.kOdometrySTDDevs,
      VisionConstants.kVisionSTDDevs);
  private final Field2d field = new Field2d();

  // Creates a new DriveSubsystem.
  public DriveSubsystem() {
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    swerveModulePositions = new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition()
    };

   poseEstimator.update(Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle), swerveModulePositions);

    field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("gyro angle", gyro.getAngle());
    SmartDashboard.putNumber("odometryX", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("odometryY", poseEstimator.getEstimatedPosition().getY());

    // AdvantageScope Logging
    double[] LOG_DATA = {
        frontLeft.getPosition().angle.getDegrees(), m_frontLeft.driveOutput,
        frontRight.getPosition().angle.getDegrees(), m_frontRight.driveOutput,
        rearLeft.getPosition().angle.getDegrees(), m_rearLeft.driveOutput,
        rearRight.getPosition().angle.getDegrees(), m_rearRight.driveOutput,
    };
    SmartDashboard.putNumberArray("AdvantageScope Swerve States", logData);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
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
    // Depending on whether the robot is being driven in field relative, calculate
    // the desired states for each of the modules
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, calculatedRotation,
        Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(gyroAngle)) : 
        new ChassisSpeeds(xSpeed, ySpeed, calculatedRotation));

    setModuleStates(swerveModuleStates);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Robot.isReal() ? gyro.getRotation2d() : new Rotation2d(m_gyroAngle),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
    gyroAngle = 0;
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    poseEstimator.addVisionMeasurement(pose, timestamp);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);

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
    gyroAngle += DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates).omegaRadiansPerSecond
        * Robot.kDefaultPeriod;
  }

}
