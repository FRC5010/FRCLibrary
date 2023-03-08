package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.PivotSubsystem;

public class PivotPower extends CommandBase {
    private PivotSubsystem elevatorSubsystem;
    private Supplier<Double> moveSpeed;

    public PivotPower(PivotSubsystem elevatorSubsystem, Supplier<Double> moveSpeed) {
        this.moveSpeed = moveSpeed;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(this.elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Elevator Move");
    }

    @Override 
    public void execute() {
        // double currentPosition = this.elevatorSubsystem.getPivotPosition();
        // double speed = (this.moveSpeed.get());
        // double newPosition = currentPosition + speed;

        // if (newPosition > 1H.0) {newPosition = 1.0;}
        // if (newPosition < -1.0) {newPosition = -1.0;}

        // this.elevatorSubsystem.setPivotPosition(newPosition);
        
        if (!elevatorSubsystem.isPivotMax() || !elevatorSubsystem.isPivotMin()){
            double speed = this.moveSpeed.get(); 
            elevatorSubsystem.pivotPow(speed, true);
        } 
    }

    @Override
    public void end(boolean interrupted) {
        this.elevatorSubsystem.stopAndHoldPivot();;
    }

    @Override
    public boolean isFinished() {
        return false;
    }    
    
}