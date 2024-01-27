package frc.robot.commands;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Chasis;

public class Cmd_Intake extends Command {
  //Variables para control por Joystick del chasis
  private final Sub_Chasis Chasis;
  private double Setpoint;

  //Constructor
  public Cmd_Intake(Sub_Chasis sub_chasis, double setpoint) {
    //Referencia a variables locales y requerimientos
    addRequirements(sub_chasis);
    this.Chasis = sub_chasis;
    this.Setpoint=setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Chasis.IntakeSpeed(Setpoint); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
