// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.Constants;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SlewRateLimiter;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftDriveMotorReversed);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kRearLeftDriveMotorReversed);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightDriveMotorReversed);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kRearRightDriveMotorReversed);

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  private double m_gyroAngle;

  private final Timer m_headingCorrectionTimer = new Timer();
  private final PIDController m_headingCorrectionPID = new PIDController(DriveConstants.kPHeadingCorrectionController,
      0, 0);
  private SwerveModulePosition[] m_swerveModulePositions = new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
  };

  private SwerveModuleState[] m_desiredStates;

  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(), m_swerveModulePositions, new Pose2d(), VisionConstants.kOdometrySTDDevs,
      VisionConstants.kVisionSTDDevs);

  private final Field2d m_field = new Field2d();
  private final Field2d m_flippedField = new Field2d();

  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(DriveConstants.kMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter m_rotationSpeedLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAccelerationUnitsPerSecond);

  /** Creates a new DriveSubsystem. */
  @SuppressWarnings("unused")
  public DriveSubsystem() {
    this.zeroHeading();
    this.resetOdometry(new Pose2d());
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("flipped field", m_flippedField);
    m_headingCorrectionTimer.restart();
    m_headingCorrectionPID.enableContinuousInput(-Math.PI, Math.PI);

    // TODO: Set a custom crop window for improved performance (the bot only needs
    // to see april tags on the reef)
    m_poseEstimator.setVisionMeasurementStdDevs(VisionConstants.kVisionSTDDevs);

    if (VisionConstants.kUseVision && Robot.isReal()) {
      if (VisionConstants.kUseLeftLL) {
        LimelightHelpers.setCameraPose_RobotSpace(
            VisionConstants.kLimelightNameLeft,
            VisionConstants.kCamPosLeft.getX(),
            VisionConstants.kCamPosLeft.getY(),
            VisionConstants.kCamPosLeft.getZ(),
            VisionConstants.kCamPosLeft.getRotation().getX(),
            VisionConstants.kCamPosLeft.getRotation().getY(),
            VisionConstants.kCamPosLeft.getRotation().getZ());
        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightNameLeft, VisionConstants.kIMUMode);

        LimelightHelpers.setCropWindow(VisionConstants.kLimelightNameLeft, -1, 1, -1, 0.4);
      }

      if (VisionConstants.kUseRightLL) {
        LimelightHelpers.setCameraPose_RobotSpace(
          VisionConstants.kLimelightNameRight,
          VisionConstants.kCamPosRight.getX(),
          VisionConstants.kCamPosRight.getY(),
          VisionConstants.kCamPosRight.getZ(),
          VisionConstants.kCamPosRight.getRotation().getX(),
          VisionConstants.kCamPosRight.getRotation().getY(),
          VisionConstants.kCamPosRight.getRotation().getZ());
        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightNameRight, VisionConstants.kIMUMode);

        LimelightHelpers.setCropWindow(VisionConstants.kLimelightNameRight, -1, 1, -1, 0.4);
      }
    }

    m_desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds());
  }

  @SuppressWarnings("unused")
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_swerveModulePositions = new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };

    m_poseEstimator.update(Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle),
        m_swerveModulePositions);

    measureLimelight(VisionConstants.kLimelightNameLeft, VisionConstants.kUseLeftLL);
    measureLimelight(VisionConstants.kLimelightNameRight, VisionConstants.kUseRightLL);

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("gyro angle", m_gyro.getAngle());
    SmartDashboard.putNumber("odometryX", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("odometryY", m_poseEstimator.getEstimatedPosition().getY());

    // AdvantageScope Logging
    // max speed = 1 (for ease of use in AdvantageScope)
    double[] logData = {
        m_frontLeft.getPosition().angle.getDegrees(), m_frontLeft.driveOutput,
        m_frontRight.getPosition().angle.getDegrees(), m_frontRight.driveOutput,
        m_rearLeft.getPosition().angle.getDegrees(), m_rearLeft.driveOutput,
        m_rearRight.getPosition().angle.getDegrees(), m_rearRight.driveOutput,
    };

    double[] logDataDesired = {
      m_desiredStates[0].angle.getDegrees(), m_desiredStates[0].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
      m_desiredStates[1].angle.getDegrees(), m_desiredStates[1].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
      m_desiredStates[2].angle.getDegrees(), m_desiredStates[2].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
      m_desiredStates[3].angle.getDegrees(), m_desiredStates[3].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
    };

    SmartDashboard.putNumberArray("AdvantageScope Swerve Desired States", logDataDesired);
    SmartDashboard.putNumberArray("AdvantageScope Swerve States", logData);

    Pose2d flippedPose = AllianceFlipUtil.apply(getPose());
    m_flippedField.setRobotPose(flippedPose);
    SmartDashboard.putNumber("flip X", flippedPose.getX());
    SmartDashboard.putNumber("flip Y", flippedPose.getY());
    SmartDashboard.putNumber("flip Rot", flippedPose.getRotation().getDegrees());
  }

  public void measureLimelight(String name, boolean useLimelight) {

    if (!useLimelight) {
      return;
    }

    boolean LLreal = LimelightHelpers.getLatency_Pipeline(name) != 0.0;
    if (VisionConstants.kUseVision && Robot.isReal() && LLreal) {
      // Update LimeLight with current robot orientation
      LimelightHelpers.SetRobotOrientation(name, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);

      // Get the pose estimate
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

      // Add it to your pose estimator if it is a valid measurement
      if (limelightMeasurement != null && limelightMeasurement.tagCount != 0 && m_gyro.getRate() < 720) {
        m_poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds);
      }
    }

    SmartDashboard.putBoolean(name + " valid", LLreal);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
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
    // If we are rotating, reset the timer
    if (rotation != 0) {
      m_headingCorrectionTimer.reset();
    }

    /*
     * Heading correction helps maintain the same heading and
     * prevents rotational drive while our robot is translating
     * 
     * For heading correction we use a timer to ensure that we
     * lose all rotational momentum before saving the heading
     * that we want to maintain
     */

    // TODO: Test heading correction without timer
    // TODO: Test heading correction using gyro's rotational velocity (if it is 0
    // then set heading instead of timer)

    // Save our desired rotation to a variable we can add our heading correction
    // adjustments to
    double calculatedRotation = rotation;

    double currentAngle = MathUtil.angleModulus(m_gyro.getRotation2d().getRadians());

    // If we are not translating or if not enough time has passed since the last
    // time we rotated
    if ((xSpeed == 0 && ySpeed == 0)
        || m_headingCorrectionTimer.get() < DriveConstants.kHeadingCorrectionTurningStopTime) {
      // Update our desired angle
      m_headingCorrectionPID.setSetpoint(currentAngle);
    } else {
      // If we are translating or if we have not rotated for a long enough time
      // then maintain our desired angle
      calculatedRotation = m_headingCorrectionPID.calculate(currentAngle);
    }

    // TODO: set speed limiter rate based on elevator height using interlocks/constraints
    // m_xSpeedLimiter.setRateLimit(DriveConstants.kMaxAccelerationUnitsPerSecond);
    // m_ySpeedLimiter.setRateLimit(DriveConstants.kMaxAccelerationUnitsPerSecond);
    // m_rotationSpeedLimiter.setRateLimit(DriveConstants.kMaxAngularAccelerationUnitsPerSecond);

    /**
    * To prevent jerks when swapping to robot relative driving, 
    * we convert our robot relative speeds to field relative early
    * so the previous value stored in the slew rate limiter filter is always valid
    */

    // If we are not driving in field relative, then convert our robot relative speeds to field relative
    if (!fieldRelative) {
      Translation2d fieldRelativeTranslation = new Translation2d(xSpeed, ySpeed).rotateBy(Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle));

      xSpeed = fieldRelativeTranslation.getX();
      ySpeed = fieldRelativeTranslation.getY();
      // rotation doesn't need to be updated because it is the same in both field and robot relative
    }

    xSpeed = m_xSpeedLimiter.calculate(xSpeed);
    ySpeed = m_ySpeedLimiter.calculate(ySpeed);
    calculatedRotation = m_rotationSpeedLimiter.calculate(calculatedRotation);

    // Depending on whether the robot is being driven in field relative, calculate
    // the desired states for each of the modules
    m_desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, calculatedRotation,
          Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle)));

    SwerveDriveKinematics.desaturateWheelSpeeds(m_desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetOdometry(pose, false);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   * @param ignoreRotation True if rotation in pose should be ignored (i.e. use gyro)
   */
  public void resetOdometry(Pose2d pose, boolean ignoreRotation) {
    Rotation2d rot = Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle);

    if (AllianceFlipUtil.shouldFlip()) {
        rot = new Rotation2d(rot.getRadians() + Math.PI);
    }

    m_poseEstimator.resetPosition(
        rot,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        ignoreRotation ? new Pose2d(pose.getTranslation(), rot) : pose);
  }

  /**
   * Sets the current heading to zero
   * @param heading heading in radians
   */
  public void zeroHeading() {
    zeroHeading(0);
  }

  /**
   * Sets the current heading to heading
   * @param heading heading in radians
   */
  public void zeroHeading(double heading) {
    m_gyro.reset(); // does not actually need to be the correct value. all uses only care about rate
    m_gyroAngle = heading;
    resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d(heading)), false);
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(pose, timestamp);
  }

  /** Sets the module states every 10ms (100Hz), faster than the regular periodic loop */
  public void fastPeriodic() {
    m_frontLeft.setDesiredState(m_desiredStates[0]);
    m_frontRight.setDesiredState(m_desiredStates[1]);
    m_rearLeft.setDesiredState(m_desiredStates[2]);
    m_rearRight.setDesiredState(m_desiredStates[3]);

    // Takes the integral of the rotation speed to find the current angle for the
    // simulator
    m_gyroAngle += DriveConstants.kDriveKinematics.toChassisSpeeds(m_desiredStates).omegaRadiansPerSecond
        * Constants.kFastPeriodicPeriod;
  }

  public void autonDrive(ChassisSpeeds speeds) {
    m_desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_desiredStates);
  }
}
