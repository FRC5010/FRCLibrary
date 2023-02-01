// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.FRC5010.Vision.VisionSystem;
import frc.robot.FRC5010.constants.GenericSwerveConstants;
import frc.robot.FRC5010.drive.pose.DrivetrainPoseEstimator;
import frc.robot.FRC5010.drive.pose.SwervePose;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;

/** Add your docs here. */
public class    SwerveDrivetrain extends GenericDrivetrain{

    private ChassisSpeeds chassisSpeeds;

    private GenericSwerveModule frontLeft, frontRight, backLeft, backRight;

    private GenericGyro gyro; 

    private GenericSwerveConstants swerveConstants;

    public SwerveDrivetrain(Mechanism2d mechVisual, GenericSwerveModule frontLeft, GenericSwerveModule frontRight, GenericSwerveModule backLeft, GenericSwerveModule backRight, GenericGyro genericGyro, VisionSystem visonSystem, GenericSwerveConstants swerveConstants) {
        super(mechVisual);

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.swerveConstants = swerveConstants;

        this.gyro = genericGyro;
        this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        poseEstimator = new DrivetrainPoseEstimator(new SwervePose(gyro, swerveConstants.getKinematics(), frontLeft, frontRight, backLeft, backRight), visonSystem);

        new Thread(() -> {
            try{
              Thread.sleep(1000);
            }catch(Exception e){}
            zeroGyroscope();
          }).start();
        //TODO Auto-generated constructor stub
    }

    @Override
    public void drive(ChassisSpeeds direction) {
        this.chassisSpeeds = direction;
    }

    public void zeroGyroscope() {
        gyro.setAngle(0);
    }
    

    public void setModuleStates(SwerveModuleState[] setDesiredStates){
        SwerveModuleState[] states = setDesiredStates;
        SwerveDriveKinematics.desaturateWheelSpeeds(states, swerveConstants.getkPhysicalMaxSpeedMetersPerSecond());
        
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);

        // if(states[0].speedMetersPerSecond > 0.0001){
        //         frontLeft.setState(states[0]);
        //         frontRight.setState(states[1]);
        //         backLeft.setState(states[2]);
        //         backRight.setState(states[3]);
        // }else{
        //         frontLeft.setState(new SwerveModuleState(0, new Rotation2d(45*5)));
        //         frontRight.setState(new SwerveModuleState(0, new Rotation2d(45*7)));
        //         backLeft.setState(new SwerveModuleState(0, new Rotation2d(45*3)));
        //         backRight.setState(new SwerveModuleState(0, new Rotation2d(45)));
        // }
    }

    public void stop(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public GenericSwerveConstants getSwerveConstants() {
        return swerveConstants;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = swerveConstants.getKinematics().toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
        poseEstimator.update();
        // super.periodic();

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
}
