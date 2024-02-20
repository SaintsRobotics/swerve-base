// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */  
public class ShuffleboardWrapperBase {
    public static ShuffleboardWrapperBase m_default = new ShuffleboardWrapperBase("Default");

    private String m_name;

    private static boolean checkForTitle(String title, ShuffleboardLayout lay) {
        for (ShuffleboardComponent<?> com : lay.getComponents()) {
            if (com.getTitle() == title) return false;
        }
        return true;
    }

    private ShuffleboardWrapperBase(String name) {
        m_name = name;
        ShuffleboardLayout lay = Shuffleboard.getTab(m_name).getLayout("Match Info", BuiltInLayouts.kList);
        if (checkForTitle("Timer", lay))
            lay.addNumber("Timer", () -> MathUtil.clamp(DriverStation.getMatchTime(), 0, 255));
        if (checkForTitle("Enabled", lay))
            lay.addBoolean("Enabled", () -> DriverStation.isEnabled());
        if (checkForTitle("Driver Controller", lay)) // Assumes port 0 driver
            lay.addBoolean("Driver Controller ", () -> DriverStation.isJoystickConnected(0));
        if (checkForTitle("Operator Controller", lay)) // Assumes port 1 driver
            lay.addBoolean("Operator Controller", () -> DriverStation.isJoystickConnected(1));
    }

    public void addSubsystem(DriveSubsystem system) {
        ShuffleboardLayout lay = Shuffleboard.getTab(m_name).getLayout("Drive", BuiltInLayouts.kList);
        lay.withSize(5, 5);
        if (checkForTitle("X-Pos", lay))
            lay.addNumber("X-Pos", () -> system.getPose().getX());
        if (checkForTitle("Y-Pos", lay))
            lay.addNumber("Y-Pos", () -> system.getPose().getY());
        if (checkForTitle("Rot", lay))
            lay.addNumber("Rot", () -> system.getPose().getRotation().getRadians());
        if (checkForTitle("AdvantageScope Swerve States", lay))
            lay.addDoubleArray("AdvantageScope Swerve States", () -> system.getLogDataC());
        if (checkForTitle("heading correction timer value", lay))
            lay.addNumber("heading correction timer value", () -> system.getHeadingCorrectionTimer());
        if (checkForTitle("AdvantageScope Swerve Desired States", lay))
            lay.addDoubleArray("AdvantageScope Swerve Desired States", () -> system.getLogDataD());

        lay = Shuffleboard.getTab(m_name).getLayout("Field", BuiltInLayouts.kList);
        if (checkForTitle("Field", lay))
            lay.add("Field", system.getField());
    }

    protected void addSubsystem(Subsystem system) {
        
    }
}
