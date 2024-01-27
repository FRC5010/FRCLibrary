// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FRC5010.Vision.AprilTags;
import frc.robot.FRC5010.Vision.VisionMultiCam;
import frc.robot.FRC5010.constants.GenericMechanism;
import frc.robot.FRC5010.motors.MotorController5010;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.crescendo.commands.RunPivot;
import frc.robot.crescendo.commands.RunShooter;

/** Add your docs here. */
public class CompBot_2024 extends GenericMechanism {

        MotorController5010 pivotMotor;
        PivotSubsystem pivotSubsystem;
        ShooterSubsystem shooterSubsystem;
        MotorController5010 topShooterMotor;
        MotorController5010 bottomShooterMotor;
        VisionMultiCam visionSystem;
        

        public CompBot_2024(Mechanism2d visual, ShuffleboardTab displayTab) {
                super(visual, displayTab);

                // Motor Setup
                pivotMotor = MotorFactory.NEO(10);

                pivotSubsystem = new PivotSubsystem(pivotMotor, visual);

                topShooterMotor = MotorFactory.NEO(11);
                bottomShooterMotor = MotorFactory.NEO(12);
                shooterSubsystem = new ShooterSubsystem(visual, topShooterMotor, bottomShooterMotor);

                visionSystem = new VisionMultiCam("Vision", 0, AprilTags.aprilTagFieldLayout);

                visionSystem.addLimeLightCameraAngle("orange", 0.3556, -10, 0, 1, null);
        }

       

        @Override
        public void configureButtonBindings(Controller driver, Controller operator) {
            driver.setLeftYAxis(driver.createLeftYAxis().deadzone(0.07).negate());
            driver.setRightYAxis(driver.createRightYAxis().deadzone(0.07).negate());

        }

        @Override
        public void setupDefaultCommands(Controller driver, Controller operator) {
            pivotSubsystem.setDefaultCommand(new RunPivot(() -> driver.getLeftYAxis(), pivotSubsystem));
            shooterSubsystem.setDefaultCommand(new RunShooter(() -> driver.getRightYAxis(), shooterSubsystem));
 
            
        }

        @Override
        protected void initRealOrSim() {
                if (RobotBase.isReal()) {
                        

                }
        }

        @Override
        public void initAutoCommands() {
            //AutoBuilder.configureCustom(null, null, null);
        }

        public void disabledBehavior() {
                
        }

        @Override
        public Command generateAutoCommand(Command autoCommand) {
                return null;
        }
}
