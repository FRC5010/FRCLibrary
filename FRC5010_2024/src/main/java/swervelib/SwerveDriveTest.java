package swervelib;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.encoders.SwerveAbsoluteEncoder;

/**
 * Class to perform tests on the swerve drive.
 */
public class SwerveDriveTest
{

  /**
   * Set the angle of the modules to a given {@link Rotation2d}
   *
   * @param swerveDrive {@link SwerveDrive} to use.
   * @param moduleAngle {@link Rotation2d} to set every module to.
   */
  public static void angleModules(SwerveDrive swerveDrive, Rotation2d moduleAngle)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.setDesiredState(new SwerveModuleState(0, moduleAngle), false, true);
    }
  }

  /**
   * Power the drive motors for the swerve drive to a set duty cycle percentage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param percentage  Duty cycle percentage of voltage to send to drive motors.
   */
  public static void powerDriveMotorsDutyCycle(SwerveDrive swerveDrive, double percentage)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getDriveMotor().set(percentage);
    }
  }

  /**
   * Power the angle motors for the swerve drive to a set percentage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param percentage  DutyCycle percentage to send to angle motors.
   */
  public static void powerAngleMotors(SwerveDrive swerveDrive, double percentage)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getAngleMotor().set(percentage);
    }
  }

  /**
   * Power the drive motors for the swerve drive to a set voltage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param volts       DutyCycle percentage of voltage to send to drive motors.
   */
  public static void powerDriveMotorsVoltage(SwerveDrive swerveDrive, double volts)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getDriveMotor().setVoltage(volts);
    }
  }

  /**
   * Power the angle motors for the swerve drive to a set voltage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param volts       Voltage to send to angle motors.
   */
  public static void powerAngleMotorsVoltage(SwerveDrive swerveDrive, double volts)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getAngleMotor().setVoltage(volts);
    }
  }

  /**
   * Set the modules to center to 0.
   *
   * @param swerveDrive Swerve Drive to control.
   */
  public static void centerModules(SwerveDrive swerveDrive)
  {
    angleModules(swerveDrive, Rotation2d.fromDegrees(0));
  }

  /**
   * Find the minimum amount of power required to move the swerve drive motors.
   *
   * @param swerveDrive      {@link SwerveDrive} to control.
   * @param minMovement      Minimum amount of movement to drive motors.
   * @param testDelaySeconds Time in seconds for the motor to move.
   * @param maxVolts         The maximum voltage to send to drive motors.
   * @return minimum voltage required.
   */
  public static double findDriveMotorKV(SwerveDrive swerveDrive, double minMovement, double testDelaySeconds,
                                        double maxVolts)
  {
    double[] startingEncoders = new double[4];
    double   kV               = 0;

    SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, 0);
    SwerveModule[] modules = swerveDrive.getModules();
    for (int i = 0; i < modules.length; i++)
    {
      startingEncoders[i] = Math.abs(modules[i].getDriveMotor().getPosition());
    }

    for (double kV_new = 0; kV_new < maxVolts; kV_new += 0.0001)
    {

      SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, kV);
      boolean foundkV          = false;
      double  startTimeSeconds = Timer.getFPGATimestamp();
      while ((Timer.getFPGATimestamp() - startTimeSeconds) < testDelaySeconds && !foundkV)
      {
        for (int i = 0; i < modules.length; i++)
        {
          if ((modules[i].getDriveMotor().getPosition() - startingEncoders[i]) > minMovement)
          {
            foundkV = true;
            break;
          }
        }
      }
      if (foundkV)
      {
        SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, 0);
        kV = kV_new;
      }
    }
    return kV;
  }

  /**
   * Find the coupling ratio for all modules.
   *
   * @param swerveDrive {@link SwerveDrive} to operate with.
   * @param volts       Voltage to send to angle motors to spin.
   * @param automatic   Attempt to automatically spin the modules.
   * @return Average coupling ratio.
   */
  public static double findCouplingRatio(SwerveDrive swerveDrive, double volts, boolean automatic)
  {
    System.out.println("Stopping the Swerve Drive.");
    SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, 0);
    SwerveDriveTest.powerAngleMotorsVoltage(swerveDrive, 0);
    Timer.delay(1);
    double couplingRatioSum = 0;
    for (SwerveModule module : swerveDrive.getModules())
    {
      if (module.getAbsoluteEncoder() == null)
      {
        throw new RuntimeException("Absolute encoders are required to find the coupling ratio.");
      }
      SwerveAbsoluteEncoder absoluteEncoder = module.getAbsoluteEncoder();
      if (absoluteEncoder.readingError)
      {
        throw new RuntimeException("Absolute encoder encountered a reading error please debug.");
      }
      System.out.println("Fetching the current absolute encoder and drive encoder position.");
      module.getAngleMotor().setVoltage(0);
      Timer.delay(1);
      Rotation2d startingAbsoluteEncoderPosition = Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
      double driveEncoderPositionRotations = module.getDriveMotor().getPosition() /
                                             module.configuration.conversionFactors.drive;
      if (automatic)
      {
        module.getAngleMotor().setVoltage(volts);
        Timer.delay(0.01);
        System.out.println("Rotating the module 360 degrees");
        while (!Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition()).equals(startingAbsoluteEncoderPosition))
          ;
        module.getAngleMotor().setVoltage(0);
      } else
      {
        DriverStation.reportWarning(
            "Spin the " + module.configuration.name + " module 360 degrees now, you have 1 minute.\n",
            false);
        Timer.delay(60);
      }
      double couplingRatio = (module.getDriveMotor().getPosition() / module.configuration.conversionFactors.drive) -
                             driveEncoderPositionRotations;
      DriverStation.reportWarning(module.configuration.name + " Coupling Ratio: " + couplingRatio, false);
      couplingRatioSum += couplingRatio;
    }
    DriverStation.reportWarning("Average Coupling Ratio: " + (couplingRatioSum / 4.0), false);
    return (couplingRatioSum / 4.0);
  }

  private static final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private static final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private static final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
  private static final MutableMeasure<Angle> m_rotations = mutable(Rotations.of(0));
  private static final MutableMeasure<Velocity<Angle>> m_angVelocity = mutable(RotationsPerSecond.of(0));

  private static SysIdRoutine sysIdRoutine;

  public static Config createConfigCustomTimeout(double timeout) {
    return new Config(null, null, Seconds.of(timeout));
  }

  public static void logDriveMotorActivity(SwerveModule module, SysIdRoutineLog log) {
    log.motor("drive-" + module.moduleNumber)
        .voltage(
            m_appliedVoltage.mut_replace(
                module.getDriveMotor().getVoltage(), Volts))
        .linearPosition(m_distance.mut_replace(module.getPosition().distanceMeters, Meters))
        .linearVelocity(
            m_velocity.mut_replace(module.getDriveMotor().getVelocity(), MetersPerSecond));

  }

  public static SysIdRoutine setDriveSysIdRoutine(Config config, SubsystemBase swerveSubsystem,
      SwerveDrive swerveDrive) {
    return new SysIdRoutine(config, new SysIdRoutine.Mechanism(
        (Measure<Voltage> voltage) -> {
          SwerveDriveTest.centerModules(swerveDrive);
          SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, voltage.in(Volts));
        },
        log -> {
          for (SwerveModule module : swerveDrive.getModules()) {
            logDriveMotorActivity(module, log);
          }
        }, swerveSubsystem));
  }

  public static void logAngularMotorActivity(SwerveModule module, SysIdRoutineLog log) {
    log.motor("angle-" + module.moduleNumber)
        .voltage(
            m_appliedVoltage.mut_replace(
                module.getDriveMotor().getVoltage(), Volts))
        .angularPosition(
            m_rotations.mut_replace(module.getAbsoluteEncoder().getAbsolutePosition(), Rotations))
        .angularVelocity(m_angVelocity.mut_replace(module.getAngleMotor().getVelocity(),
            RotationsPerSecond));
  }

  public static SysIdRoutine setAngleSysIdRoutine(Config config, SubsystemBase swerveSubsystem,
      SwerveDrive swerveDrive) {
    return new SysIdRoutine(config, new SysIdRoutine.Mechanism(
        (Measure<Voltage> voltage) -> {
          SwerveDriveTest.powerAngleMotors(swerveDrive, voltage.in(Volts));
          SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, 0);
  
        },
        log -> {
          for (SwerveModule module : swerveDrive.getModules()) {
            logAngularMotorActivity(module, log);
          }
        }, swerveSubsystem));
  }

  public static Command generateSysIdCommand(SysIdRoutine sysIdRoutine, double delay, double quasiTimeout, double dynamicTimeout) {
    return Commands.waitSeconds(quasiTimeout).deadlineWith(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward))
        .andThen(Commands.waitSeconds(delay))
        .andThen(Commands.waitSeconds(quasiTimeout).deadlineWith(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)))
        .andThen(Commands.waitSeconds(delay))
        .andThen(Commands.waitSeconds(dynamicTimeout).deadlineWith(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)))
        .andThen(Commands.waitSeconds(delay))
        .andThen(Commands.waitSeconds(dynamicTimeout).deadlineWith(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)));
  }
}
