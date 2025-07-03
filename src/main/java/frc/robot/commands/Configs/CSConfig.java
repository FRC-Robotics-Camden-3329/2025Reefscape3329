package frc.robot.commands.Configs;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class CSConfig extends ParallelCommandGroup {
    public CSConfig(Elevator elevator, Coral coral, Algae algae) {
        addCommands(
            elevator.moveElevatorCommand(Constants.ElevatorConstants.CS),
            coral.moveCoralCommand(Constants.CoralConstants.CS),
            algae.moveAlgaeCommand(Constants.AlgaeConstants.CS)
        );
    }
}
