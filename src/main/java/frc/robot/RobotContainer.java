package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Cmd_Gyro;
import frc.robot.commands.Cmd_Intake;
import frc.robot.commands.Cmd_ManualDriveChasis;
import frc.robot.commands.Cmd_MoveChasis;
import frc.robot.subsystems.Sub_Chasis;
import frc.robot.commands.Visioncmd;


public class RobotContainer {
  CommandXboxController ChasisControl = new CommandXboxController(0);
  CommandXboxController SubsystemControl = new CommandXboxController(1);
  Sub_Chasis chasis = new Sub_Chasis();



  public RobotContainer() {
    chasis.setDefaultCommand(new Cmd_ManualDriveChasis(chasis, () -> ChasisControl.getRightTriggerAxis(), () -> ChasisControl.getLeftTriggerAxis(), () -> ChasisControl.getLeftX(), () -> ChasisControl.back().getAsBoolean(), ()->ChasisControl.getRightX()));
    configureBindings();
  }


  private void configureBindings() {
    //ChasisControl.a().whileTrue(new Command);
    //ChasisControl.a().whileTrue(new Cmd_MoveChasis(chasis, 100));

    ChasisControl.a().whileTrue(new Cmd_Intake(chasis,-0.2));
    ChasisControl.b().whileTrue(new Cmd_Intake(chasis,1));
    ChasisControl.y().whileTrue(new Visioncmd(chasis));
    //ChasisControl.b().whileTrue(new Cmd_Gyro(chasis,90));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
