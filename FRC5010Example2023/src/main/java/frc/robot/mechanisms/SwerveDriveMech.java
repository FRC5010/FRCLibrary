// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.FRC5010.Controller;
import frc.robot.FRC5010.GenericMechanism;
import frc.robot.FRC5010.VisionSystem;
import frc.robot.FRC5010.drive.GenericSwerveModule;
import frc.robot.FRC5010.drive.SwerveDrivetrain;
import frc.robot.FRC5010.drive.ThriftySwerveModule;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.AnalogInput5010;
import frc.robot.FRC5010.sensors.NavXGyro;
import frc.robot.commands.JoystickToSwerve;

/** Add your docs here. */
public class SwerveDriveMech extends GenericMechanism {

    GenericSwerveModule frontLeft, frontRight, backLeft, backRight;
    SwerveDrivetrain swerveDrive;
    VisionSystem visonSystem; 

    NavXGyro gyro = new NavXGyro(SPI.Port.kMXP);

    public SwerveDriveMech(Mechanism2d robotMechSim, VisionSystem visonSystem) {
        super(robotMechSim);

        this.visonSystem = visonSystem; 

        frontLeft = new ThriftySwerveModule(robotMechSim.getRoot("frontleft", 45, 15), "frontleft", new NEO(1), new NEO(2), SwerveDrivetrain.kFrontLeftAbsoluteOffsetRad, new AnalogInput5010(0));  

        frontRight = new ThriftySwerveModule(robotMechSim.getRoot("frontright", 45, 45), "frontright", new NEO(7), new NEO(8), SwerveDrivetrain.kFrontRightAbsoluteOffsetRad, new AnalogInput5010(1));

        backLeft = new ThriftySwerveModule(robotMechSim.getRoot("backleft", 15, 15), "backleft", new NEO(3), new NEO(4), SwerveDrivetrain.kBackLeftAbsoluteOffsetRad, new AnalogInput5010(2));

        backRight = new ThriftySwerveModule(robotMechSim.getRoot("backright", 15, 45), "backright", new NEO(5), new NEO(6), SwerveDrivetrain.kBackRightAbsoluteOffsetRad, new AnalogInput5010(3));

        swerveDrive = new SwerveDrivetrain(robotMechSim, frontLeft, frontRight, backLeft, backRight, gyro, visonSystem);

        
        //TODO Auto-generated constructor stub
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        // TODO Auto-generated method stub
        driver.setLeftXAxis(driver.createLeftXAxis().deadzone(0.075).negate().rate(SwerveDrivetrain.kTeleDriveMaxAccelerationUnitsPerSecond));
        driver.setLeftYAxis(driver.createLeftYAxis().deadzone(0.075).negate().rate(SwerveDrivetrain.kTeleDriveMaxAccelerationUnitsPerSecond));
        driver.setRightXAxis(driver.createRightXAxis().deadzone(0.075).negate().rate(SwerveDrivetrain.kTeleDriveMaxAngularAccelerationUnitsPerSecond));

        swerveDrive.setDefaultCommand(new JoystickToSwerve(swerveDrive, 
        () -> driver.getLeftYAxis(), 
        () -> driver.getLeftXAxis(), 
        () -> driver.getRightXAxis(), 
        () -> driver.createAButton().getAsBoolean()));
        
    }

    @Override
    public void setupDefaultCommands() {

        // TODO Auto-generated method stub
    }

    @Override
    protected void initRealOrSim() {
        // TODO Auto-generated method stub
        
    }}
