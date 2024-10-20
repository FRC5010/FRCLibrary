// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chargedup;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5010.common.arch.GenericMechanism;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.hardware.MotorModelConstants;
import org.frc5010.common.sensors.ButtonBoard;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.subsystems.LedSubsystem;
import frc.robot.chargedup.commands.HomeElevator;
import frc.robot.chargedup.commands.HomePivot;
import frc.robot.chargedup.commands.IntakeSpin;
import frc.robot.chargedup.commands.MoveElevator;
import frc.robot.chargedup.commands.PivotElevator;

/** Add your docs here. */
public class ChargedUpMech extends GenericMechanism {
	private ElevatorSubsystem elevatorSubsystem;
	private IntakeSubsystem intakeSubsystem;
	private PivotSubsystem pivotSubsystem;
	private ButtonBoard buttonOperator;
	private final double kElevatorMaxSpeedLimit = 1;
	private final double kElevatorMinSpeedLimit = 0.25;
	private final double kSpinMaxSpeed = 0.6;
	private final double kSpinLowSpeed = 0.35;
	private double speedLimit = kElevatorMaxSpeedLimit;
	private double intakeSpeedLimit = kSpinMaxSpeed;
	private boolean ledConePickUp = false;
	private LedSubsystem ledSubsystem;

	private ElevatorLevel elevatorLevel = ElevatorLevel.ground;

	public ChargedUpMech(Mechanism2d robotMechVisual, ShuffleboardTab shuffleTab, ButtonBoard buttonOperator,
			LedSubsystem ledSubsystem) {
		super(robotMechVisual, shuffleTab);
		// use this to PID the Elevator
		// https://www.chiefdelphi.com/t/is-tuning-spark-max-smart-motion-impossible/404104/2
		this.elevatorSubsystem = new ElevatorSubsystem(
				MotorFactory.NEO(11), new GenericPID(RobotBase.isReal() ? 8 : 30, 0, 0),
				new MotorModelConstants(1, 1, 1),
				mechVisual, 0, 3, () -> pivotSubsystem.getPivotPosition());

		this.pivotSubsystem = new PivotSubsystem(
				MotorFactory.NEO(9),
				new GenericPID(RobotBase.isReal() ? 12 : 0.047, 0.0, 0.03),
				new MotorModelConstants(1, 8.6825, 1),
				1, 8,
				() -> elevatorSubsystem.getExtendPosition(), mechVisual);

		this.intakeSubsystem = new IntakeSubsystem(
				MotorFactory.NEO(19),
				MotorFactory.NEO(18),
				new MotorModelConstants(0, 0, 0),
				new GenericPID(0.003, 0, 0),
				new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1),
				robotMechVisual);
		this.buttonOperator = buttonOperator;
		this.ledSubsystem = ledSubsystem;
		initRealOrSim();
	}

	@Override
	public void configureButtonBindings(Controller driver, Controller operator) {

		driver.setRightTrigger(driver.createRightTrigger());
		driver.setLeftTrigger(driver.createLeftTrigger());
		operator.setRightTrigger(operator.createRightTrigger());
		operator.setLeftTrigger(operator.createLeftTrigger());

		// Move to current extend level
		buttonOperator.getButton(1)
				.whileTrue(new MoveElevator(elevatorSubsystem, () -> elevatorLevel));

		operator.createUpPovButton().whileTrue(new MoveElevator(elevatorSubsystem, () -> elevatorLevel));

		buttonOperator.getButton(2)
				.whileTrue(new HomeElevator(elevatorSubsystem, pivotSubsystem));

		operator.createDownPovButton().whileTrue(new HomeElevator(elevatorSubsystem, pivotSubsystem));

		buttonOperator.getButton(6)
				.onTrue(
						new SequentialCommandGroup(
								new InstantCommand(() -> {
									elevatorLevel = ElevatorLevel.ground;
								}),
								new PivotElevator(pivotSubsystem, ElevatorLevel.ground)

						));

		operator.createAButton().onTrue(
				new SequentialCommandGroup(
						new InstantCommand(() -> {
							elevatorLevel = ElevatorLevel.ground;
						}),
						new PivotElevator(pivotSubsystem, ElevatorLevel.ground)

				));

		buttonOperator.getButton(5)
				.onTrue(
						new SequentialCommandGroup(
								new InstantCommand(
										() -> elevatorLevel = ElevatorLevel.loading),
								new PivotElevator(pivotSubsystem, ElevatorLevel.loading)

						));

		operator.createXButton().onTrue(
				new SequentialCommandGroup(
						new InstantCommand(
								() -> elevatorLevel = ElevatorLevel.loading),
						new PivotElevator(pivotSubsystem, ElevatorLevel.loading)

				));

		buttonOperator.getButton(4)
				.onTrue(
						new SequentialCommandGroup(
								new InstantCommand(
										() -> elevatorLevel = ElevatorLevel.medium),
								new PivotElevator(pivotSubsystem, ElevatorLevel.medium)

						));
		operator.createYButton().onTrue(
				new SequentialCommandGroup(
						new InstantCommand(
								() -> elevatorLevel = ElevatorLevel.medium),
						new PivotElevator(pivotSubsystem, ElevatorLevel.medium)

				));

		buttonOperator.getButton(3)
				.onTrue(
						new SequentialCommandGroup(
								new InstantCommand(
										() -> elevatorLevel = ElevatorLevel.high),
								new PivotElevator(pivotSubsystem, ElevatorLevel.high)));

		operator.createBButton().onTrue(
				new SequentialCommandGroup(
						new InstantCommand(
								() -> elevatorLevel = ElevatorLevel.high),
						new PivotElevator(pivotSubsystem, ElevatorLevel.high)));

		buttonOperator.getButton(7)
				.onTrue(new InstantCommand(() -> {
					speedLimit = kElevatorMinSpeedLimit;
					intakeSpeedLimit = kSpinLowSpeed;
				}))
				.onFalse(new InstantCommand(() -> {
					intakeSpeedLimit = kSpinMaxSpeed;
					speedLimit = kElevatorMaxSpeedLimit;
				}));

			buttonOperator.getButton(8).onTrue(new InstantCommand(() -> {
				intakeSubsystem.setIntakeCone();
			}, intakeSubsystem));

			operator.createLeftBumper().onTrue(new InstantCommand(() -> {
				intakeSubsystem.setIntakeCone();
			}, intakeSubsystem));

			buttonOperator.getButton(9).onTrue(new InstantCommand(() -> {
				intakeSubsystem.setIntakeCube();
			}, intakeSubsystem));

			operator.createRightBumper().onTrue(new InstantCommand(() -> {
				intakeSubsystem.setIntakeCube();
			}, intakeSubsystem));

			buttonOperator.getButton(10)
					.whileTrue(new IntakeSpin(intakeSubsystem, () -> Math.max(-intakeSpeedLimit,
							-1)));

			new Trigger(() -> (Math
					.abs(driver.getRightTrigger() - driver.getLeftTrigger()) > 0.01))
					.whileTrue(new IntakeSpin(intakeSubsystem,
							() -> (driver.getRightTrigger()
									- driver.getLeftTrigger()) * 1));

			new Trigger(() -> (Math
					.abs(operator.getRightTrigger()) > 0.01))
					.whileTrue(new IntakeSpin(intakeSubsystem,
							() -> (operator.getRightTrigger() * -1)));

		buttonOperator.setYAxis(buttonOperator.createYAxis().negate().deadzone(0.05));
		buttonOperator.setXAxis(buttonOperator.createXAxis().deadzone(0.05)); // The deadzone isnt technically

		driver.createUpPovButton().whileTrue(new HomePivot(pivotSubsystem));
		// necessary
		// but I have seen self movement
		// without
		// it

		// new Trigger(() -> (Math.abs(buttonOperator.getXAxis()) > 0.01))
		// .onTrue(new ElevatorPower(elevatorSubsystem, () -> (buttonOperator.getXAxis()
		// * speedLimit))
		// );

		// new Trigger(() -> (Math.abs(buttonOperator.getYAxis()) > 0.01))
		// .onTrue(new PivotPower(pivotSubsystem, () -> (buttonOperator.getYAxis() *
		// speedLimit))
		// );

		driver.createDownPovButton()
				.onTrue(new InstantCommand(() -> pivotSubsystem.toggleOverride(), pivotSubsystem));

		driver.createXButton().onTrue(new InstantCommand(() -> {
			elevatorSubsystem.toggleOverride();
			pivotSubsystem.toggleOverride();
		}, elevatorSubsystem, pivotSubsystem));

		operator.setRightYAxis(operator.createRightYAxis().deadzone(0.2).negate());
		operator.setLeftYAxis(operator.createLeftYAxis().deadzone(0.2));

		driver.createRightBumper().onTrue(new InstantCommand(() -> ledSubsystem.togglePickUp(), ledSubsystem));
	}

	public IntakeSubsystem getIntakeSubsystem() {
		return intakeSubsystem;
	}

	public ElevatorSubsystem getElevatorSubsystem() {
		return elevatorSubsystem;
	}

	public PivotSubsystem getPivotSubsystem() {
		return pivotSubsystem;
	}

	@Override
	public void setupDefaultCommands(Controller driver, Controller operator) {
		elevatorSubsystem.setDefaultCommand(new FunctionalCommand(
				() -> {
				},
				() -> {
					elevatorSubsystem.extendPow(buttonOperator.getXAxis() * speedLimit
							- operator.getLeftYAxis());

				},
				(Boolean interrupted) -> {
					elevatorSubsystem.extendPow(0);
				},
				() -> false,
				elevatorSubsystem));
		pivotSubsystem.setDefaultCommand(new FunctionalCommand(
				() -> {
				},
				() -> {
					pivotSubsystem.pivotPow(buttonOperator.getYAxis() * speedLimit
							+ operator.getRightYAxis(), true);
				},
				(Boolean interrupted) -> {
					pivotSubsystem.pivotPow(0, true);
				},
				() -> false,
				pivotSubsystem));

		// ledSubsystem.setDefaultCommand(new LedDefaultCommand(ledSubsystem,
		// intakeSubsystem, elevatorSubsystem));
	}

	@Override
	public void disabledBehavior() {
		ledSubsystem.setRainbow();
	}

	@Override
	protected void initRealOrSim() {
		if (RobotBase.isSimulation()) {
			this.intakeSubsystem = new IntakeSubsystem(
					MotorFactory.NEO(19),
					MotorFactory.NEO(18),
					new MotorModelConstants(0, 0, 0),
					new GenericPID(0.003, 0, 0),
					new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1),
					mechVisual);
		}
	}

	@Override
	public void initAutoCommands() {
		// NamedCommands.registerCommand("Pivot to Ground", new
		// PivotElevator(pivotSubsystem, ElevatorLevel.ground));
		// NamedCommands.registerCommand("Pivot to High", new
		// PivotElevator(pivotSubsystem, ElevatorLevel.high));
		// NamedCommands.registerCommand("Auto Note Pickup", new InstantCommand(() -> {
		// }));
		// NamedCommands.registerCommand("Auto Note Shoot", new InstantCommand(() -> {
		// }));
	}

	@Override
	public Command generateAutoCommand(Command autoCommand) {
		// TODO Auto-generated method stub
		return autoCommand;
	}
}
