// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionMultiCam;
import frc.robot.FRC5010.constants.GenericMechanism;
import frc.robot.FRC5010.constants.SwerveConstants;
import frc.robot.FRC5010.mechanisms.Drive;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.FRC5010.sensors.gyro.GenericGyro;
import frc.robot.FRC5010.sensors.gyro.PigeonGyro;
import frc.robot.crescendo.commands.RunClimb;
import frc.robot.crescendo.commands.RunIntake;
import frc.robot.crescendo.commands.RunPivot;
import frc.robot.crescendo.commands.RunShooter;

/** Add your docs here. */
public class CompBot_2024 extends GenericMechanism {
        private SwerveConstants swerveConstants;
        private MotorController5010 pivotMotor;
        private PivotSubsystem pivotSubsystem;
        private ShooterSubsystem shooterSubsystem;
        private MotorController5010 topShooterMotor;
        private MotorController5010 bottomShooterMotor;
        private MotorController5010 feederShooterMotor;
        private IntakeSubsystem intakeSubsystem;
        private MotorController5010 bottomIntakeMotor;
        private MotorController5010 topIntakeMotor;


        private VisionMultiCam visionSystem;
        private GenericGyro gyro;
        private ClimbSubsystem climbSubsystem;
        private MotorController5010 leftClimbMotor;
        private MotorController5010 rightClimbMotor;
        private Drive drive;

        public CompBot_2024(Mechanism2d visual, ShuffleboardTab displayTab) {
                super(visual, displayTab);

                // Motor Setup
                pivotMotor = MotorFactory.NEO(10);
                topShooterMotor = MotorFactory.NEO(11);
                bottomShooterMotor = MotorFactory.NEO(12);
                leftClimbMotor = MotorFactory.NEO(0); // TODO: Add correct port
                rightClimbMotor = MotorFactory.NEO(1); // TODO: Add correct port
                feederShooterMotor = MotorFactory.NEO(13);
                topIntakeMotor = MotorFactory.NEO(14);
                bottomIntakeMotor = MotorFactory.NEO(15);

                gyro = new PigeonGyro(15);

                visionSystem = new VisionMultiCam("Vision", 0, AprilTags.aprilTagFieldLayout);
                visionSystem.addLimeLightCameraAngle("orange", 0.3556, -10, 0, 1, null);
                pivotSubsystem = new PivotSubsystem(pivotMotor, mechVisual);
                shooterSubsystem = new ShooterSubsystem(mechVisual, topShooterMotor, bottomShooterMotor, feederShooterMotor);
                climbSubsystem = new ClimbSubsystem(leftClimbMotor, rightClimbMotor, gyro, mechVisual);
                intakeSubsystem = new IntakeSubsystem(topIntakeMotor, bottomIntakeMotor, mechVisual);
                
                //swerveConstants = new SwerveConstants(Units.inchesToMeters(Constants.Physical.TRACK_WIDTH_INCHES), Units.inchesToMeters(Constants.Physical.WHEEL_BASE_INCHES));

                //drive = new Drive(visionSystem, gyro, Drive.Type.YAGSL_SWERVE_DRIVE, null, swerveConstants, "2024_compbot");

                
                
        }


        @Override
        public void configureButtonBindings(Controller driver, Controller operator) {
                driver.setLeftYAxis(driver.createLeftYAxis().deadzone(0.07).negate());
                driver.setRightYAxis(driver.createRightYAxis().deadzone(0.07).negate());
                driver.setRightTrigger(driver.createRightTrigger().deadzone(0.07));
                driver.setLeftTrigger(driver.createLeftTrigger().deadzone(0.07));

                operator.setLeftTrigger(operator.createLeftTrigger());
                operator.setRightTrigger(operator.createRightTrigger());
                operator.setLeftYAxis(operator.createLeftYAxis().deadzone(0.07).negate());
                operator.setRightYAxis(operator.createRightYAxis().deadzone(0.07).negate());

                driver.createXButton().onTrue(Commands.runOnce(() -> pivotSubsystem.setPivotPosition(180), pivotSubsystem));
                driver.createBButton().onTrue(Commands.runOnce(() -> pivotSubsystem.setReference(pivotSubsystem.TRAP_LEVEL), pivotSubsystem));
                driver.createAButton().onTrue(Commands.runOnce(() -> pivotSubsystem.setReference(pivotSubsystem.HOME_LEVEL), pivotSubsystem));
                driver.createYButton().onTrue(Commands.runOnce(() -> pivotSubsystem.setReference(pivotSubsystem.AMP_LEVEL), pivotSubsystem));


        }

        @Override
        public void setupDefaultCommands(Controller driver, Controller operator) {
                pivotSubsystem.setDefaultCommand(new RunPivot(() -> driver.createUpPovButton().getAsBoolean() ? 0.2 : (driver.createDownPovButton().getAsBoolean() ? -0.2 : 0), pivotSubsystem)); // TODO: Rewrite Messy Ternary Operator
                shooterSubsystem.setDefaultCommand(new RunShooter(() -> operator.getRightTrigger(), () -> operator.getLeftTrigger(), shooterSubsystem));
                climbSubsystem.setDefaultCommand(new RunClimb(() -> operator.getLeftYAxis(), () -> operator.getRightYAxis(), climbSubsystem));
                intakeSubsystem.setDefaultCommand(new RunIntake(() -> driver.getRightTrigger() != 0 ? driver.getRightTrigger() : -driver.getLeftTrigger(), intakeSubsystem));
        }

        @Override
        public void setupTestDefaultCommmands(Controller driver, Controller operator) {
                driver.createAButton().whileTrue(shooterSubsystem.getTopSysIdRoutineCommand());
                driver.createBButton().whileTrue(shooterSubsystem.getBottomSysIdRoutineCommand());
                driver.createYButton().whileTrue(shooterSubsystem.getFeederSysIdRoutineCommand());
                driver.createXButton().whileTrue(pivotSubsystem.getSysIdCommand());
                driver.createLeftBumper().whileTrue(intakeSubsystem.getBottomSysIdRoutineCommand());
                driver.createRightBumper().whileTrue(intakeSubsystem.getTopSysIdRoutineCommand());
        }

        @Override
        protected void initRealOrSim() {
                if (RobotBase.isReal()) {

                }
        }

        @Override
        public void initAutoCommands() {
                // AutoBuilder.configureCustom(null, null, null);
        }

        public void disabledBehavior() {

        }

        @Override
        public Command generateAutoCommand(Command autoCommand) {
                return null;
        }
}
