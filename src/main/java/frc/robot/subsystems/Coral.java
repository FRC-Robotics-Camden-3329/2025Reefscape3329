package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase{
    ProfiledPIDController pid;
    SparkMax pivot, intake;
    DutyCycleEncoder encoder;
    double target;
    public Coral(){
        pid = new ProfiledPIDController(Constants.CoralConstants.kP,
                                        Constants.CoralConstants.kI,
                                        Constants.CoralConstants.kD,
                                        new Constraints(Constants.CoralConstants.maxVelocity,
                                                        Constants.CoralConstants.maxAcceleration));
        pivot = new SparkMax(Constants.CoralConstants.pivotID, MotorType.kBrushless);
        intake = new SparkMax(Constants.CoralConstants.intakeID, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(Constants.CoralConstants.encoderID);
        setTarget(0.27);
    }

    public void runPivot(double speed){
        pivot.set(speed);
    }

    public void setTarget(double degrees){
        target = degrees;
        pid.setGoal(target);
    }

    public double getEncoderPosition(){
        return encoder.get() - 0.1227;
    }

    public boolean isAtPosition(){
        return pid.atGoal();
    }

    public Command moveCoralCommand(double degrees){
        return Commands.runOnce(() -> this.setTarget(degrees)).andThen(Commands.waitSeconds(0.5))
                                .until(() -> isAtPosition());
    }

    public void runIntake(double speed){
        intake.set(-speed);
    }

    public Command runIntakeCommand(double speed){
        return Commands.runOnce(() -> this.runIntake(speed));
    }
        
    public Command intakeCoralCommand(){
        return this.runEnd(() -> this.runIntake(Constants.CoralConstants.intakeSpeed), () -> this.runIntake(0.1));
    }

    public Command ejectCoralCommand(){
        return this.runEnd(() -> this.runIntake(-Constants.CoralConstants.ejectSpeed), () -> this.runIntake(0));
    }

    public Command stopCoralCommand(){
        return this.runEnd(() -> this.runIntake(0), () -> this.runIntake(0));
    }

    @Override
    public void periodic(){
        runPivot(pid.calculate(getEncoderPosition()));
        SmartDashboard.putNumber("Coral Position", getEncoderPosition());
        SmartDashboard.putNumber("Coral Target", target);
        SmartDashboard.putBoolean("Coral At Position", isAtPosition());
    }
}
