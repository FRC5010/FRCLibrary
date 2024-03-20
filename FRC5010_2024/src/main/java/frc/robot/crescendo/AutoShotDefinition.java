// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public enum AutoShotDefinition {
    STAGE_SHOT_LONG(4.50, 4.70);
    

    private final double x;
    private final double y;
    private final double angle;
    private final double CRESCENDO_FIELD_WIDTH = 16.579;

    private AutoShotDefinition(double x, double y) {
        this.x = x;
        this.y = y;
        this.angle = 0;
    }

    private AutoShotDefinition(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.angle = theta;
    }

    public Pose2d getPose(Alliance alliance) {
        return new Pose2d(alliance == Alliance.Blue ? x : CRESCENDO_FIELD_WIDTH-x, y, Rotation2d.fromDegrees(angle));
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, Rotation2d.fromDegrees(angle));
    }

}
