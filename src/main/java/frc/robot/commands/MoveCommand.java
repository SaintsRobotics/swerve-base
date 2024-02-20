// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;

public class MoveCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;

  private DoubleSupplier m_slowMoSupplier;
	private DoubleSupplier m_xSpeedSupplier;
	private DoubleSupplier m_ySpeedSupplier;
	private DoubleSupplier m_rotSpeedSupplier;
  private BooleanSupplier m_fieldRelativeSupplier;


  /** Creates a new MoveCommand. */
  public MoveCommand(DriveSubsystem subsystem) {
    m_driveSubsystem = subsystem;
    addRequirements(m_driveSubsystem);

    m_slowMoSupplier = () -> 0;
	  m_xSpeedSupplier = () -> 0;
	  m_ySpeedSupplier = () -> 0;
    m_rotSpeedSupplier = () -> 0;
    m_fieldRelativeSupplier = () -> true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(m_xSpeedSupplier.getAsDouble(), m_ySpeedSupplier.getAsDouble(),
        m_rotSpeedSupplier.getAsDouble(), m_fieldRelativeSupplier.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Default Command should always return false
  @Override
  public boolean isFinished() {
    return false;
  }

  private double inputToSpeed(DoubleSupplier xyz){
    return MathUtil.applyDeadband(
      -xyz.getAsDouble(), 
      IOConstants.kControllerDeadband) 
      * DriveConstants.kMaxSpeedMetersPerSecond 
      * (1-m_slowMoSupplier.getAsDouble() 
      * IOConstants.kSlowModeScalar) 
      * IOConstants.kControllerSensitivity;
  }

	/**
	 * Sets the X speed of the robot using a {@link DoubleSupplier}.
	 * 
	 * @param x {@link DoubleSupplier} that returns the X speed.
	 * @return This, for method chaining.
	 */
	public MoveCommand withXSpeedSupplier(DoubleSupplier x) {
		m_xSpeedSupplier = () -> {
      return inputToSpeed(x);
    };

    return this;
	}

	/**
	 * Sets the Y speed of the robot using a {@link DoubleSupplier}.
	 * 
	 * @param y {@link DoubleSupplier} that returns the Y speed.
	 * @return This, for method chaining.
	 */
	public MoveCommand withYSpeedSupplier(DoubleSupplier y) {
		m_ySpeedSupplier = () -> {
      return inputToSpeed(y);
    };

		return this;
	}

	/**
	 * Sets the rotational speed of the robot using a {@link DoubleSupplier}.
	 * 
	 * @param rot {@link DoubleSupplier} that returns the rotational speed.
	 * @return This, for method chaining.
	 */
	public MoveCommand withRotSpeedSupplier(DoubleSupplier rot) {
		m_rotSpeedSupplier = () -> {
      return inputToSpeed(rot);
    };		

    return this;
	}

  /**
	 * Sets whether the robot drives in field relative mode vis a vis {@link BoolSupplier}.
	 * 
	 * @param fieldRelative {@link BooleanSupplier} that returns whether the robot is in field relative mode
	 * @return This, for method chaining.
	 */
  public MoveCommand withFieldRelativeSupplier(BooleanSupplier fieldRelative){
    m_fieldRelativeSupplier = fieldRelative;
    return this;
  }

  /*
   * Sets whether the robot drives in slow mo vis a vis {@link }.
   * 
   * @param triggerAxis {@link DoubleSupplier} that returns whether the robot is
   * in slow mo mode
   * 
   * @return This, for method chaining.
   */
  public MoveCommand withSlowMoSupplier(DoubleSupplier triggerAxis){
    m_slowMoSupplier = triggerAxis;
    return this;
  }
}
