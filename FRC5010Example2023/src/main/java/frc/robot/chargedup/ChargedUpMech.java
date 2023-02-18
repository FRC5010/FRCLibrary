// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

        this.intakeSubsystem = new IntakeSubsystem(
                MotorFactory.NEO(18), 
                MotorFactory.NEO(19), 
                new MotorModelConstants(0, 0, 0), 
                new GenericPID(0, 0, 0), 
                new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1), 
                robotMechVisual
        );
        // TODO: Set up IntakeSubsystem add correct values please
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        operator.createYButton()
                .onTrue(new ElevatorMove(elevatorSubsystem, () -> 0.5));
        operator.createAButton()
                .onTrue(new ElevatorMove(elevatorSubsystem, () -> -0.5));
        operator.createBButton()
                .whileTrue(new ElevatorOut(elevatorSubsystem, () -> -0.1));
        operator.createXButton()
                .whileTrue(new ElevatorOut(elevatorSubsystem, () -> 0.1));


        // new Trigger(() -> (Math.abs(driver.createRightTrigger().get() - driver.createLeftTrigger().get()) > 0.01))
        //         .onTrue(new IntakeSpin(intakeSubsystem, () -> driver.createRightTrigger().get() - driver.createLeftTrigger().get()));
 
        // operator.createRightBumper()
        //         .onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeCone(), intakeSubsystem));
        // operator.createLeftBumper()
        //         .onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeCube(), intakeSubsystem));
        // operator.createStartButton()
        //         .onTrue(new IntakeSpin(intakeSubsystem, () -> -0.1));

        operator.setRightYAxis(driver.createRightYAxis().deadzone(.07).negate());
        operator.setLeftYAxis(driver.createLeftYAxis().deadzone(0.07));
    }

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
        elevatorSubsystem.setDefaultCommand(new FunctionalCommand(
                () -> {
                },
                () -> {
                    //elevatorSubsystem.pivotPow(driver.getRightYAxis());
                    //elevatorSubsystem.extendPow(operator.getLeftYAxis());
                },
                (Boolean interrupted) -> {
                    elevatorSubsystem.pivotPow(0);
                    elevatorSubsystem.extendPow(0);
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
