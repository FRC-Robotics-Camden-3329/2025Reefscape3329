package frc.robot.commands.Configs;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class L2Config extends ParallelCommandGroup {
    public L2Config(Elevator elevator, Coral coral, Algae algae) {
        addCommands(
            elevator.moveElevatorCommand(Constants.ElevatorConstants.L2),
            coral.moveCoralCommand(Constants.CoralConstants.L2),
            algae.moveAlgaeCommand(Constants.AlgaeConstants.L2)
        );
    }
}
