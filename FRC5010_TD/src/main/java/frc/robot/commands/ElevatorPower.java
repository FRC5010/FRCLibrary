package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chargedup.ElevatorSubsystem;

public class ElevatorPower extends CommandBase {
    private Supplier<Double> moveSpeed;
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorPower(ElevatorSubsystem elevatorSubsystem, Supplier<Double> moveSpeed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.moveSpeed = moveSpeed;
    }

    @Override
    public void initialize() {
        System.out.println("Elevator Out");
    }

    @Override 
    public void execute() {
        double speed = (this.moveSpeed.get());
        elevatorSubsystem.extendPow(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // if (interrupted){
        //     elevatorSubsystem.stopExtend();
        //   } else {
        //     elevatorSubsystem.stopAndHoldExtend();
        //   }
        elevatorSubsystem.stopAndHoldExtend();
    }

    @Override
    public boolean isFinished() {
        return false;
    }    
}
