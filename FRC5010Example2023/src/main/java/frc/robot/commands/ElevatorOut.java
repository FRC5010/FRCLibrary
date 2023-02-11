package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.motors.hardware.NEO;
import frc.robot.FRC5010.subsystems.ElevatorSubsystem;

public class ElevatorOut extends CommandBase {
    private Supplier<Double> moveSpeed;
    private NEO moveMotor;

    public ElevatorOut(NEO moveMotor, Supplier<Double> moveSpeed) {
        this.moveMotor = moveMotor;
        this.moveSpeed = moveSpeed;
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        double speed = (this.moveSpeed.get());
        this.moveMotor.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.moveMotor.set(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }    
}
