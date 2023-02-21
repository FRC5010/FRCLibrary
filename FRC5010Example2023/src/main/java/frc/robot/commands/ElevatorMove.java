package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.ElevatorSubsystem;

public class ElevatorMove extends CommandBase {
    private ElevatorSubsystem elevatorSubsystem;
    private Supplier<Double> moveSpeed;

    public ElevatorMove(ElevatorSubsystem elevatorSubsystem, Supplier<Double> moveSpeed) {
        this.moveSpeed = moveSpeed;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        // double currentPosition = this.elevatorSubsystem.getPivotPosition();
        // double speed = (this.moveSpeed.get());
        // double newPosition = currentPosition + speed;

        // if (newPosition > 1.0) {newPosition = 1.0;}
        // if (newPosition < -1.0) {newPosition = -1.0;}

        // this.elevatorSubsystem.setPivotPosition(newPosition);

        double speed = this.moveSpeed.get(); 
        elevatorSubsystem.pivotPow(speed);

        
    }

    @Override
    public void end(boolean interrupted) {
        this.elevatorSubsystem.stopPivot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }    
    
}