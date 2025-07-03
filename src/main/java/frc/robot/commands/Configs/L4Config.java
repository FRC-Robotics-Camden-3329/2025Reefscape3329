package frc.robot.commands.Configs;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class L4Config extends ParallelCommandGroup {
    public L4Config(Elevator elevator, Coral coral, Algae algae) {
        addCommands(
            elevator.moveElevatorCommand(Constants.ElevatorConstants.L4),
            coral.moveCoralCommand(Constants.CoralConstants.L4),
            algae.moveAlgaeCommand(Constants.AlgaeConstants.L4)
        );
    }
}
