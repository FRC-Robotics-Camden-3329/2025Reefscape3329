package frc.robot.commands.Configs;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class PConfig extends ParallelCommandGroup {
    public PConfig(Elevator elevator, Coral coral, Algae algae) {
        addCommands(
            elevator.moveElevatorCommand(Constants.ElevatorConstants.P),
            coral.moveCoralCommand(Constants.CoralConstants.P),
            algae.moveAlgaeCommand(Constants.AlgaeConstants.P)
        );
    }
}
