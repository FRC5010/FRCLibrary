package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.ElevatorSubsystem;

public class ElevatorOut extends CommandBase {
    private Supplier<Double> moveSpeed;
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorOut(ElevatorSubsystem elevatorSubsystem, Supplier<Double> moveSpeed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.moveSpeed = moveSpeed;
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        double speed = (this.moveSpeed.get());
        elevatorSubsystem.elevate(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.elevate(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }    
}
