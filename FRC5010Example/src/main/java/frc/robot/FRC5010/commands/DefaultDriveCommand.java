package frc.robot.FRC5010.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.GenericDrivetrain;

public class DefaultDriveCommand extends CommandBase {
    private final GenericDrivetrain drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private MechanismRoot2d joystick;
    private MechanismLigament2d xAxis;
    private MechanismLigament2d yAxis;
    private MechanismLigament2d heading;

    public DefaultDriveCommand(GenericDrivetrain drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        
        joystick = drivetrainSubsystem.getMechVisual().getRoot("joystick", 30, 30);
        xAxis = new MechanismLigament2d("xAxis", 1, 90, 6, new Color8Bit(Color.kDarkRed));
        yAxis = new MechanismLigament2d("yAxis", 1, 180, 6, new Color8Bit(Color.kDarkSalmon));
        heading = new MechanismLigament2d("heading", 20, 0, 6, new Color8Bit(Color.kDeepPink));
        joystick.append(xAxis);
        joystick.append(yAxis);
        joystick.append(heading);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double x = m_translationXSupplier.getAsDouble();
        double y = m_translationYSupplier.getAsDouble();
        double r = m_rotationSupplier.getAsDouble();
        
        xAxis.setLength(x * 30);
        yAxis.setLength(y * 30);
        heading.setAngle(180 * r);

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        // drivetrainSubsystem.drive(
        //     ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, drivetrainSubsystem.getHeading())
        // );
        drivetrainSubsystem.drive(
            new ChassisSpeeds(x, y, r)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
