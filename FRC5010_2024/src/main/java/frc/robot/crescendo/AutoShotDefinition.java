// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import java.util.Optional;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FRC5010.Vision.VisionSystem.Rotation;

/** Add your docs here. */
public enum AutoShotDefinition {
    STAGE_SHOT_LONG(4.50, 4.70),
    LEFT_LONG_SHOT(5.0, 6.7),
    CENTER_SHOT_LONG(4.0, 5.59),
    CENTER_SHOT_MID(2.81, 5.56),
    B2_SHOT(2.9, 5.55, 180, 13.0, 3000),
    C8_SHOT(1.52, 3.63),
    A1_SHOT(1.64, 7.02),
    CENTER_SHOT_SHORT(2.15, 5.55, 0),
    RIGHT_SHOT_SHORT(1.61, 4.53),
    LEFT_SHOT_SHORT(1.6, 7.3),
    LEFT_SPEAKER_SHOT(1.7, 6.30),
    RIGHT_SHOT_LONG(3.10, 2.85),
    SUBWOOFER_SHOT(1.40, 5.55, 180),
    NOTE_2(2.60, 6.90);
    

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

    private AutoShotDefinition(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.heading = theta;
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

    public AutoShotDefinition withCalculatedTargetValues(TargetingSystem targetingSystem) {
        if (pivot.isEmpty()) {
            pivot = Optional.of(TargetingSystem.interpolatePivotAngle(getPose().getTranslation().getDistance(targetingSystem.getTarget().getTranslation().toTranslation2d())));
        }
        if (shooterVelocity.isEmpty()) {
            shooterVelocity = Optional.of(TargetingSystem.interpolateShooterSpeed(getPose().getTranslation().getDistance(targetingSystem.getTarget().getTranslation().toTranslation2d())));
        }
        return this;
    }

    public Pose2d getPose(Alliance alliance) {
        double red_heading = 180 - heading;
        Rotation2d rotation_red_heading;
        if (red_heading > 180) {
            rotation_red_heading = Rotation2d.fromDegrees(red_heading-360);
        } else if (red_heading < -180) {
            rotation_red_heading = Rotation2d.fromDegrees(red_heading+360);
        } else {
            rotation_red_heading = Rotation2d.fromDegrees(red_heading);
        }
        
        return new Pose2d(alliance == Alliance.Blue ? x : CRESCENDO_FIELD_WIDTH-x, y, alliance == Alliance.Blue ? Rotation2d.fromDegrees(heading) : rotation_red_heading );
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
