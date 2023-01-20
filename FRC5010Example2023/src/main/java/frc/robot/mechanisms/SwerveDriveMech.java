// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.FRC5010.Controller;
import frc.robot.FRC5010.GenericMechanism;
import frc.robot.FRC5010.drive.GenericSwerveModule;
import frc.robot.FRC5010.drive.SwerveDrivetrain;
import frc.robot.FRC5010.drive.ThriftySwerveModule;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.sensors.AnalogInput5010;
import frc.robot.FRC5010.sensors.NavXGyro;
import frc.robot.FRC5010.sensors.RevEncoder;
import frc.robot.FRC5010.sensors.WpiEncoder;

/** Add your docs here. */
public class SwerveDriveMech extends GenericMechanism {

    GenericSwerveModule frontLeft, frontRight, backLeft, backRight;
    SwerveDrivetrain swerveDrive;

    NavXGyro gyro = new NavXGyro(Port.kMXP);

    public SwerveDriveMech(Mechanism2d robotMechSim) {
        super(robotMechSim);

        

        frontLeft = new ThriftySwerveModule(robotMechSim.getRoot("frontleft", 0, 0), "frontleft", new NEO(1), new NEO(2), SwerveDrivetrain.kFrontLeftAbsoluteOffsetRad, new AnalogInput5010(0));  

        frontRight = new ThriftySwerveModule(robotMechSim.getRoot("frontright", 0, 0), "frontright", new NEO(7), new NEO(8), SwerveDrivetrain.kFrontRightAbsoluteOffsetRad, new AnalogInput5010(1));

        backLeft = new ThriftySwerveModule(robotMechSim.getRoot("backleft", 0, 0), "backleft", new NEO(3), new NEO(4), SwerveDrivetrain.kBackLeftAbsoluteOffsetRad, new AnalogInput5010(2));

        backRight = new ThriftySwerveModule(robotMechSim.getRoot("backright", 0, 0), "backright", new NEO(5), new NEO(6), SwerveDrivetrain.kBackRightAbsoluteOffsetRad, new AnalogInput5010(3));

        swerveDrive = new SwerveDrivetrain(robotMechSim, frontLeft, frontRight, backLeft, backRight, gyro);
        //TODO Auto-generated constructor stub
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setupDefaultCommands() {
        // TODO Auto-generated method stub
        
    }

    @Override
    protected void initRealOrSim() {
        // TODO Auto-generated method stub
        
    }}
