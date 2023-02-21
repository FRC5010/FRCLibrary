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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FRC5010.constants.GenericPID;
import frc.robot.FRC5010.drive.GenericDrivetrain;
import frc.robot.FRC5010.drive.swerve.SwerveDrivetrain;
import frc.robot.FRC5010.mechanisms.GenericMechanism;
import frc.robot.FRC5010.motors.MotorFactory;
import frc.robot.FRC5010.motors.hardware.MotorModelConstants;
import frc.robot.FRC5010.sensors.ButtonBoard;
import frc.robot.FRC5010.sensors.Controller;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.ElevatorOut;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.SetElevatorExtendFromLevel;
import frc.robot.commands.SetElevatorPivotFromLevel;

/** Add your docs here. */
public class ChargedUpMech extends GenericMechanism {
    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ButtonBoard buttonOperator;
    private double speedLimit = .3;

    public ChargedUpMech(Mechanism2d robotMechVisual, ShuffleboardTab shuffleTab, ButtonBoard buttonOperator) {
        super(robotMechVisual, shuffleTab);
        this.elevatorSubsystem = new ElevatorSubsystem(
                MotorFactory.NEO(9), new GenericPID(3.6174, 0, 0),
                MotorFactory.NEO(11), new GenericPID(3.6174, 0, 0),
                new MotorModelConstants(1, 1, 1), 
                new MotorModelConstants(1, 1, 1),
                mechVisual, 0);

        this.intakeSubsystem = new IntakeSubsystem(
                MotorFactory.NEO(18), 
                MotorFactory.NEO(19), 
                new MotorModelConstants(0, 0, 0), 
                new GenericPID(0.003, 0, 0), 
                new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1), 
                robotMechVisual
        );
        // TODO: Set up IntakeSubsystem add correct values please
        this.buttonOperator = buttonOperator;
    }

    @Override
    public void configureButtonBindings(Controller driver, Controller operator) {
        
        buttonOperator.getButton(1)
                .onTrue(new SetElevatorExtendFromLevel(elevatorSubsystem));
        buttonOperator.getButton(2)
                .onTrue(new SetElevatorExtendFromLevel(elevatorSubsystem, ElevatorLevel.ground));
        buttonOperator.getButton(6)
                .onTrue(new SetElevatorPivotFromLevel(elevatorSubsystem, ElevatorLevel.ground));
        buttonOperator.getButton(5)
                .onTrue(new SetElevatorPivotFromLevel(elevatorSubsystem, ElevatorLevel.low));
        buttonOperator.getButton(4)
                .onTrue(new SetElevatorPivotFromLevel(elevatorSubsystem, ElevatorLevel.medium));
        buttonOperator.getButton(3)
                .onTrue(new SetElevatorPivotFromLevel(elevatorSubsystem, ElevatorLevel.high));
        buttonOperator.getButton(7)
                .onTrue(new InstantCommand(() -> {speedLimit = 0.15;}))
                .onFalse(new InstantCommand(() -> {speedLimit = 0.3;}));

        buttonOperator.getButton(8).onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeCone(), intakeSubsystem));
        buttonOperator.getButton(9).onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeCube(), intakeSubsystem));
        buttonOperator.getButton(10).whileTrue(new IntakeSpin(intakeSubsystem, () -> -0.5));

        buttonOperator.setYAxis(buttonOperator.createYAxis().deadzone(0.05));
        buttonOperator.setXAxis(buttonOperator.createXAxis().negate().deadzone(0.05)); //The deadzone isnt technically necessary but I have seen self movement without it

        new Trigger(() -> (Math.abs(buttonOperator.getXAxis()) > 0.01))
            .onTrue(new ElevatorOut(elevatorSubsystem, () -> (buttonOperator.getXAxis() * speedLimit))
        );

        new Trigger(() -> (Math.abs(buttonOperator.getYAxis()) > 0.01))
            .onTrue(new ElevatorMove(elevatorSubsystem, () -> (buttonOperator.getYAxis() * speedLimit))
        );

        new Trigger(() -> (Math.abs(driver.createRightTrigger().get() - driver.createLeftTrigger().get()) > 0.01))
                .whileTrue(new IntakeSpin(intakeSubsystem, () -> driver.createRightTrigger().get() - driver.createLeftTrigger().get()));
 
        // operator.createRightBumper()
        //         .onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeCone(), intakeSubsystem));
        // operator.createLeftBumper()
        //         .onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeCube(), intakeSubsystem));
        // operator.createStartButton()
        //         .onTrue(new IntakeSpin(intakeSubsystem, () -> -0.1));

        operator.setRightYAxis(operator.createRightYAxis().deadzone(.07).negate());
        operator.setLeftYAxis(operator.createLeftYAxis().deadzone(0.07));
    }
    

    @Override
    public void setupDefaultCommands(Controller driver, Controller operator) {
        elevatorSubsystem.setDefaultCommand(new FunctionalCommand(
                () -> {
                },
                () -> {
                    elevatorSubsystem.pivotPow(operator.getRightYAxis());
                    elevatorSubsystem.extendPow(operator.getLeftYAxis());
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
