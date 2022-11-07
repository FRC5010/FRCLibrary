// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionConstants;
import frc.robot.FRC5010.Vision.VisionValuesPhotonCamera;
import frc.robot.FRC5010.Vision.AprilTags.AprilTag;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class DrivetrainPoseEstimator extends SubsystemBase {
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your
  // various sensors. Smaller numbers will cause the filter to "trust" the
  // estimate from that particular
  // component more than the others. This in turn means the particualr component
  // will have a stronger
  // influence on the final pose estimate.
  Matrix<N5, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05);
  Matrix<N3, N1> localMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));
  Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));
  private Drivetrain driveTrain;
  private VisionSystem vision;
  private final Field2d field2d = new Field2d();

  private final DifferentialDrivePoseEstimator m_poseEstimator;
  public DrivetrainPoseEstimator(Drivetrain driveTrain, VisionSystem vision) {
    this.driveTrain = driveTrain;
    this.vision = vision;
    m_poseEstimator = new DifferentialDrivePoseEstimator(
        driveTrain.getGyroRotation2d(),
        new Pose2d(),
        stateStdDevs,
        localMeasurementStdDevs,
        visionMeasurementStdDevs);
    ShuffleboardTab tab = Shuffleboard.getTab("Pose");
    tab.addString("Pose (X,Y)", this::getFormattedPose).withPosition(0, 4);
    tab.addNumber("Pose Degrees", () -> getCurrentPose().getRotation().getDegrees()).withPosition(1, 4);
    tab.add(field2d);

    for (AprilTag at: AprilTags.aprilTagPoses) {
      if (at.pose.getX() != 0 && at.pose.getY() != 0 && at.pose.getZ() != 0) {
        field2d.getObject(at.fieldDescriptor).setPose(at.pose.toPose2d());
      }
    }
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f)",
        Units.metersToInches(pose.getX()),
        Units.metersToInches(pose.getY()));
  }

  public Pose2d getCurrentPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Perform all periodic pose estimation tasks.
   *
   * @param actWheelSpeeds Current Speeds (in m/s) of the drivetrain wheels
   * @param leftDist       Distance (in m) the left wheel has traveled
   * @param rightDist      Distance (in m) the right wheel has traveled
   */
  public void update() {
    Pose3d camPose = vision.getRawValues().getCameraPose();
    if (null != camPose) {
      System.out.println("CamPose: X: " + camPose.getX() + " Y: " + camPose.getY() + " Z:" + camPose.getZ() + " R: " + Units.radiansToDegrees(camPose.getRotation().getAngle()));
      Pose2d robotPoseEst = camPose.transformBy(VisionConstants.kCameraToRobot).toPose2d();
      double imageCaptureTime = Timer.getFPGATimestamp() - vision.getRawValues().getLatency() / 1000.0;
      
      field2d.getObject("MyRobot" + ((VisionValuesPhotonCamera)vision.getRawValues()).getFiducialId()).setPose(robotPoseEst);    
      System.out.println("RobotPoseEst: X: " + robotPoseEst.getX() + " Y: " + robotPoseEst.getY() + " R: " + robotPoseEst.getRotation().getDegrees());
      m_poseEstimator.addVisionMeasurement(robotPoseEst, imageCaptureTime);
    }
    DifferentialDriveWheelSpeeds actWheelSpeeds = new DifferentialDriveWheelSpeeds(driveTrain.getLeftEncoderRate(),
        driveTrain.getRightEncoderRate());
    double leftDist = driveTrain.getLeftDistance();
    double rightDist = driveTrain.getRightDistance();
    m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), driveTrain.getGyroRotation2d(), actWheelSpeeds, leftDist, rightDist);

    field2d.setRobotPose(getCurrentPose());
  }

  /**
   * Force the pose estimator to a particular pose. This is useful for indicating
   * to the software
   * when you have manually moved your robot in a particular position on the field
   * (EX: when you
   * place it on the field at the start of the match).
   *
   * @param pose
   */
  public void resetToPose(Pose2d pose) {
    m_poseEstimator.resetPosition(pose, driveTrain.getGyroRotation2d());
  }

  /** @return The current best-guess at drivetrain position on the field. */
  public Pose2d getPoseEst() {
    return m_poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    update();
  }
}
