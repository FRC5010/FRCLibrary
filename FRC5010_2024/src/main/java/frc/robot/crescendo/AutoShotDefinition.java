// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import java.util.Optional;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public enum AutoShotDefinition {
    STAGE_SHOT_LONG(4.50, 4.70),
    LEFT_LONG_SHOT(5.0, 6.7),
    CENTER_SHOT_LONG(4.0, 5.59),
    B2_SHOT(2.89, 5.56, 180, 19.5,0);
    

    private final double x;
    private final double y;
    private final double heading;
    private Optional<Double> pivot;
    private Optional<Double> shooterVelocity;
    private final double CRESCENDO_FIELD_WIDTH = 16.579;

    private AutoShotDefinition(double x, double y) {
        this.x = x;
        this.y = y;
        this.heading = 0;
        this.pivot = Optional.empty();
        this.shooterVelocity = Optional.empty();
    }

    private AutoShotDefinition(double x, double y, double theta, double pivot, double shooterVelocity) {
        this.x = x;
        this.y = y;
        this.heading = theta;
        this.pivot = Optional.of(pivot);
        this.shooterVelocity = Optional.of(shooterVelocity);
    }

    public Pose2d getPose(Alliance alliance) {
        return new Pose2d(alliance == Alliance.Blue ? x : CRESCENDO_FIELD_WIDTH-x, y, alliance == Alliance.Blue ? Rotation2d.fromDegrees(heading) : Rotation2d.fromDegrees(180 - heading));
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, Rotation2d.fromDegrees(heading));
    }

    public double getPivotAngle() {
        if (pivot.isEmpty()) {
            Pose3d defaultTarget = TargetingSystem.getDefaultTarget();
            pivot = Optional.of(TargetingSystem.interpolatePivotAngle(defaultTarget.getTranslation().toTranslation2d().getDistance(getPose().getTranslation())));
        }
        return pivot.get();
    }

    public double getYawAngle() {
        return heading;
    }

    public double getShooterSpeed() {
        if (shooterVelocity.isEmpty()) {
            Pose3d defaultTarget = TargetingSystem.getDefaultTarget();
            shooterVelocity = Optional.of(TargetingSystem.interpolateShooterSpeed(defaultTarget.getTranslation().toTranslation2d().getDistance(getPose().getTranslation())));
        }
        return shooterVelocity.get();
    }




}
