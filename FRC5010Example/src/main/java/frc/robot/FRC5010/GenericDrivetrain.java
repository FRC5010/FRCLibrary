// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class GenericDrivetrain extends SubsystemBase {
    private Mechanism2d mechVisual;
    public GenericDrivetrain(Mechanism2d mechVisual) {
        this.mechVisual = mechVisual;
    }
    public Mechanism2d getMechVisual() { return mechVisual; }
    public abstract void drive(ChassisSpeeds direction);
    public abstract Rotation2d getHeading();

}
