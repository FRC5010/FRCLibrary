// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.motors.hardware.MotorModelConstants;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.ElevatorOut;
import frc.robot.commands.IntakeToggle;

/** Add your docs here. */
public class ChargedUpMech extends GenericMechanism {
    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public ChargedUpMech(Mechanism2d robotMechVisual, ShuffleboardTab shuffleTab) {
        super(robotMechVisual, shuffleTab);
        this.elevatorSubsystem = new ElevatorSubsystem(
                MotorFactory.NEO(9), new GenericPID(0, 0, 0),
                MotorFactory.NEO(11), new GenericPID(0, 0, 0),
                new MotorModelConstants(1, 1, 1), 
                new MotorModelConstants(1, 1, 1),
                mechVisual);
        // TODO: Set up IntakeSubsystem once built
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        driver.createYButton()
                .onTrue(new ElevatorMove(elevatorSubsystem, () -> 0.1));
        driver.createAButton()
                .onTrue(new ElevatorMove(elevatorSubsystem, () -> -0.1));
        driver.createBButton()
                .whileTrue(new ElevatorOut(elevatorSubsystem, () -> -0.1));
        driver.createXButton()
                .whileTrue(new ElevatorOut(elevatorSubsystem, () -> 0.1));
        //TODO: Add finish intake commands
        //operator.createBButton()
                //.onTrue(new IntakeToggle(intakeSubsystem));
        //operator.createLeftTrigger();
                //.onTrue(new IntakeSpin(intakeSubsystem, () -> -0.1));
        //operator.createRightTrigger();
                //.onTrue(new IntakeSpin(intakeSubsystem, () -> 0.1));
        operator.setRightYAxis(driver.createRightYAxis().deadzone(.07).negate());
        operator.setLeftYAxis(driver.createLeftYAxis().deadzone(0.07));
    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
        elevatorSubsystem.setDefaultCommand(new FunctionalCommand(
                () -> {
                },
                () -> {
                    elevatorSubsystem.winch(driver.getRightYAxis());
                    elevatorSubsystem.elevate(driver.getLeftYAxis());
                },
                (Boolean interrupted) -> {
                    elevatorSubsystem.winch(0);
                    elevatorSubsystem.elevate(0);
                },
                () -> false,
                elevatorSubsystem));
    }

    @Override
    protected void initRealOrSim() {
    }

    @Override
    public Map<String, Command> initAutoCommands() {
        return new HashMap<>();
    }
}
