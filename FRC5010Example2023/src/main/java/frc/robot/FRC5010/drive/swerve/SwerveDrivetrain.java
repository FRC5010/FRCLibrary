// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive.swerve;

import java.util.HashMap;

import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.constants.Persisted;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.drive.GenericDrivetrain;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.drive.pose.SwervePose;
import frc.robot.FRC5010.mechanisms.DriveConstantsDef;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;

/** Add your docs here. */
public class SwerveDrivetrain extends GenericDrivetrain{

    private ChassisSpeeds chassisSpeeds;

    private GenericSwerveModule frontLeft, frontRight, backLeft, backRight;

    private GenericGyro gyro; 

    private SwerveConstants swerveConstants;

    private boolean ready = false;
    private Persisted<Double> maxChassisVelocity;

    public SwerveDrivetrain(Mechanism2d mechVisual, 
        GenericSwerveModule frontLeft, GenericSwerveModule frontRight, GenericSwerveModule backLeft, GenericSwerveModule backRight, 
        GenericGyro genericGyro, VisionSystem visonSystem, SwerveConstants swerveConstants) {
        super(mechVisual);

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.swerveConstants = swerveConstants;

        this.gyro = genericGyro;
        this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        poseEstimator = new DrivetrainPoseEstimator(new SwervePose(gyro, swerveConstants.getKinematics(), frontLeft, frontRight, backLeft, backRight), visonSystem);
        maxChassisVelocity = new Persisted<>(DriveConstantsDef.MAX_CHASSIS_VELOCITY, Double.class);

        setDrivetrainPoseEstimator(poseEstimator);

        gyro.reset();
    }

    @Override
    public void drive(ChassisSpeeds direction) {
        chassisSpeeds = direction; // for driving in simulation
        // Pose2d robotPoseVel = new Pose2d(direction.vxMetersPerSecond * 0.02, 
        //     direction.vyMetersPerSecond * 0.02, Rotation2d.fromRadians(direction.omegaRadiansPerSecond * 0.02));
        // Twist2d twistVel = getPoseEstimator().getCurrentPose().log(robotPoseVel);
        // chassisSpeeds = new ChassisSpeeds(twistVel.dx / 0.02, twistVel.dy / 0.02, twistVel.dtheta / 0.02);
        SwerveModuleState[] states = swerveConstants.getKinematics().toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
    }
    

    public void setModuleStates(SwerveModuleState[] setDesiredStates){
        SwerveModuleState[] states = setDesiredStates;
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxChassisVelocity.get());
        
        // TODO get swerve stop lock working
        // if(Math.abs(states[0].speedMetersPerSecond) < 0.001){
        //     states[0] = new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45*5)));
        //     states[1] = new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45*7)));
        //     states[2] = new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45*3)));
        //     states[3] = new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45*5)));
        // }

        // TODO add the isReady function so wheels won't move till turned close enough
        boolean isReady = true;
        isReady &= frontLeft.setState(states[0], ready);
        isReady &= frontRight.setState(states[1], ready);
        isReady &= backLeft.setState(states[2], ready);
        isReady &= backRight.setState(states[3], ready);
        ready = isReady;
    }

    public void stop(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public SwerveConstants getSwerveConstants() {
        return swerveConstants;
    }

    @Override
    public void simulationPeriodic() {
        Pose2d pose = poseEstimator.getCurrentPose();
        Transform2d direction = new Transform2d(
            new Translation2d(chassisSpeeds.vxMetersPerSecond * 0.02, chassisSpeeds.vyMetersPerSecond * 0.02), 
            new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * 0.02));
        pose = pose.transformBy(direction);
        poseEstimator.resetToPose(pose);
    }

    public BaseAutoBuilder setAutoBuilder(HashMap<String, Command> eventMap){ 
        return new SwerveAutoBuilder(
            () -> getPoseEstimator().getCurrentPose(), // Pose2d supplier
            (Pose2d pose) -> getPoseEstimator().resetToPose(pose), // Pose2d consumer, used to reset odometry at the beginning of auto
            getSwerveConstants().getKinematics(), // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0),// PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            this::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // The drive subsystem. Used to properly set the requirements of path following commands
        );        
    }
}