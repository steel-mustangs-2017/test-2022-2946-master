package frc.robot.commands.Old_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ControlerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class ManualIndexer extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final Joystick operatorController;
    private boolean indexerIn = false;
    private boolean indexerOut = false; 

    public ManualIndexer(Joystick operatorController, IndexerSubsystem indexerSubsystem) {
        this.operatorController = operatorController;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(indexerSubsystem);
    }


    @Override
    public void initialize() {
        indexerIn = operatorController.getRawButton(ControlerConstants.CONTROLLER_BUTTON_B_ID);
        indexerOut = operatorController.getRawButton(ControlerConstants.CONTROLLER_BUTTON_X_ID);
    }
}
