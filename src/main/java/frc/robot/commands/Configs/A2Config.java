package frc.robot.commands.Configs;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class A2Config extends ParallelCommandGroup {
    public A2Config(Elevator elevator, Coral coral, Algae algae) {
        addCommands(
            elevator.moveElevatorCommand(Constants.ElevatorConstants.A2),
            coral.moveCoralCommand(Constants.CoralConstants.A2),
            algae.moveAlgaeCommand(Constants.AlgaeConstants.A2)
        );
    }
}
