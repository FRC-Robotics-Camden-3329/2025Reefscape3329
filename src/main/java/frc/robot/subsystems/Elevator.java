package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    SparkMax left, right;
    RelativeEncoder encoder;
    double target;

    public Elevator() {
        right = new SparkMax(Constants.ElevatorConstants.rightID, MotorType.kBrushless);
        left = new SparkMax(Constants.ElevatorConstants.leftID, MotorType.kBrushless);
        encoder = left.getEncoder();
        encoder.setPosition(0);
        setTarget(0);
    }

    public void run(double speed) {
        right.set(-speed);
        left.set(speed);
    }

    public void setTarget(double degrees) {
        target = degrees;
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public boolean isAtPosition() {
        return getEncoderPosition() < target + 0.7 && getEncoderPosition() > target - 0.4; //0.3
    }

    public Command moveElevatorCommand(double degrees) {
        return Commands.runOnce(() -> this.setTarget(degrees))
                .andThen(Commands.waitSeconds(0.5))
                .until(() -> isAtPosition());
    }

    @Override
    public void periodic() {
        if (!isAtPosition() && getEncoderPosition() < target) {
            run(0.3);
        } else if (!isAtPosition() && getEncoderPosition() > target) {
            run(-0.15); //-0.1
        } else {
            run(0.07);
        }
        SmartDashboard.putNumber("Elevator Position", getEncoderPosition());
        SmartDashboard.putNumber("Elevator Target", target);
        SmartDashboard.putBoolean("Elevator At Position", isAtPosition());
    }
}
