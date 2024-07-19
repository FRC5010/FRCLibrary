package org.frc5010.common.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc5010.common.arch.Persisted;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.mechanisms.DriveConstantsDef;

public class DefaultDriveCommand extends Command {
  // TODO: Understand code
  private final GenericDrivetrain drivetrainSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private Supplier<Boolean> fieldOrientedDrive;

  private MechanismRoot2d joystick;
  private MechanismLigament2d xAxis;
  private MechanismLigament2d yAxis;
  private MechanismLigament2d heading;
  private Persisted<Double> maxChassisVelocity;
  private Persisted<Double> maxChassisRotation;

  private SlewRateLimiter xRateLimiter, yRateLimiter, thetaRateLimiter;

  public DefaultDriveCommand(
      GenericDrivetrain drivetrainSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      Supplier<Boolean> fieldOrientedDrive) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.fieldOrientedDrive = fieldOrientedDrive;
    maxChassisVelocity =
        new Persisted<Double>(DriveConstantsDef.MAX_CHASSIS_VELOCITY, Double.class);
    maxChassisRotation =
        new Persisted<Double>(DriveConstantsDef.MAX_CHASSIS_ROTATION, Double.class);
    joystick = drivetrainSubsystem.getMechVisual().getRoot("joystick", 30, 30);
    xAxis = new MechanismLigament2d("xAxis", 1, 90, 6, new Color8Bit(Color.kDarkRed));
    yAxis = new MechanismLigament2d("yAxis", 1, 180, 6, new Color8Bit(Color.kDarkSalmon));
    heading = new MechanismLigament2d("heading", 20, 0, 6, new Color8Bit(Color.kDeepPink));
    joystick.append(xAxis);
    joystick.append(yAxis);
    joystick.append(heading);

    xRateLimiter = new SlewRateLimiter(2, -2, 0);
    yRateLimiter = new SlewRateLimiter(2, -2, 0);
    thetaRateLimiter = new SlewRateLimiter(4, -4, 0);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    double x = m_translationXSupplier.getAsDouble();
    double y = m_translationYSupplier.getAsDouble();
    double r = m_rotationSupplier.getAsDouble();

    x = xRateLimiter.calculate(x);
    y = yRateLimiter.calculate(y);
    r = thetaRateLimiter.calculate(r);

    xAxis.setLength(x * 30);
    yAxis.setLength(y * 30);
    heading.setAngle(180 * r);

    if (fieldOrientedDrive.get()) {
      drivetrainSubsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              x * maxChassisVelocity.get(),
              y * maxChassisVelocity.get(),
              r * maxChassisRotation.get(),
              drivetrainSubsystem.getHeading()));
    } else {
      drivetrainSubsystem.drive(
          new ChassisSpeeds(
              x * maxChassisVelocity.get(),
              y * maxChassisVelocity.get(),
              r * maxChassisRotation.get()));
    }
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented
    // movement
    // drivetrainSubsystem.drive(
    //     ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, drivetrainSubsystem.getHeading())
    // );
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
