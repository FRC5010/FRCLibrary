package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.PivotSubsystem;

public class PivotPower extends CommandBase {
    private PivotSubsystem pivotSubsystem;
    private Supplier<Double> moveSpeed;
    private double prevSpeed;
    private double holdPos;

    public PivotPower(PivotSubsystem elevatorSubsystem, Supplier<Double> moveSpeed) {
        this.moveSpeed = moveSpeed;
        this.pivotSubsystem = elevatorSubsystem;
        addRequirements(this.pivotSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Elevator Move");
    }

    @Override
    public void execute() {
        double currentPosition = this.pivotSubsystem.getPivotPosition();

        double speed = this.moveSpeed.get() * 0.1;
        prevSpeed = speed;

        pivotSubsystem.pivotPow(speed, true);
    }

    @Override
    public void end(boolean interrupted) {
        this.pivotSubsystem.stopAndHoldPivot();
        ;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}