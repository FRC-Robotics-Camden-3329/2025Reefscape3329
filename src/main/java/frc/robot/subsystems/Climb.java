package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase{
    private SparkMax motor;
    public Climb(){
        motor = new SparkMax(Constants.ClimbConstants.motorID, SparkMax.MotorType.kBrushed);
    }

    public void run(double speed){
        motor.set(speed);
    }

    public Command climbCommand(){
        return this.runEnd(() -> this.run(Constants.CoralConstants.intakeSpeed), () -> this.run(0));
    }

    public Command reachCommand(){
        return this.runEnd(() -> this.run(-Constants.CoralConstants.intakeSpeed), () -> this.run(0));
    }
}
