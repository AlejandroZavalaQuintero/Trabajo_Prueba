package frc.robot.commands;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Chasis;

public class Cmd_ManualDriveChasis extends Command {
  //Variables para control por Joystick del chasis
  private final Sub_Chasis Chasis;
  private final Supplier<Double> RT, LT, XAxis, XFineAxis;
  private final Supplier<Boolean> B_Button;

  //Constructor
  public Cmd_ManualDriveChasis(Sub_Chasis sub_chasis, Supplier<Double> RT, Supplier<Double> LT, Supplier<Double> XAxis, Supplier<Boolean> B_Button,Supplier<Double> XFine) {
    //Referencia a variables locales y requerimientos
    addRequirements(sub_chasis);
    this.Chasis = sub_chasis; this.RT = RT; this.LT = LT; this.XAxis = XAxis; this.B_Button = B_Button; this.XFineAxis = XFine;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Chasis.resetPartialEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Variables para calcular velocidad
    double RightSpeed, LeftSpeed, Trigger, Turn, Boost, TurnFine, MoveF, MoveB;

    //Limpieza de ruido}
    MoveF=RT.get();
    MoveB=LT.get();
    if(MoveF>0.95){MoveF=0.95;}
    if(MoveF<0.15 && MoveF>-0.05){MoveF=0;}
    if(MoveF<0.05){MoveF=Math.abs(MoveF);}

    if(MoveB>0.95){MoveB=0.95;}
    if(MoveB<0.15 && MoveB>-0.05){MoveB=0;}
    if(MoveB<0.05){MoveB=Math.abs(MoveB);}

      
    

  
    Trigger = MoveF-MoveB; 
    Turn = XAxis.get(); if(Math.abs(Turn)<0.15){Turn = 0;}


    TurnFine = XFineAxis.get(); if(Math.abs(TurnFine)<0.15){TurnFine = 0;}
    
    if(Math.abs(TurnFine)<0.15){TurnFine = 0;}

    //Filtro de velocidad
    if(B_Button.get()){Boost = 0.3;}else{Boost = 1;}
    
    //Calculo de velocidad
    RightSpeed = (Trigger - (0.45*Turn))*Boost;
    LeftSpeed = (Trigger + (0.45*Turn))*Boost;

    //Set a los motores
    Chasis.setSpeed(RightSpeed, LeftSpeed);

   
    
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
