package frc.robot.commands.Configs;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class L1Config extends ParallelCommandGroup {
    public L1Config(Elevator elevator, Coral coral, Algae algae) {
        addCommands(
            elevator.moveElevatorCommand(Constants.ElevatorConstants.L1),
            coral.moveCoralCommand(Constants.CoralConstants.L1),
            algae.moveAlgaeCommand(Constants.AlgaeConstants.L1)
        );
    }
}
