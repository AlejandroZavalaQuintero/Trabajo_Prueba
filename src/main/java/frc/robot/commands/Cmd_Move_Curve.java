package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Chasis;

public class Cmd_Move_Curve extends Command {
  //Variables para un control dual de PID del chasis
  private final Sub_Chasis Chasis;
  private double Setpoint, Dt, LastDt, I_Zone;
  private double RightErrorP, RightErrorI, RightErrorD, RightLastError, RightSpeed, RightSetpoint;
  private double LeftErrorP, LeftErrorI, LeftErrorD, LeftLastError, LeftSpeed, ErrorTeta, ErrorTetaI,DifEnc, LeftSetpoint;
  private double kP, kI, kD, kG, kGI, Arco,Radio;

  //Constructor
  public Cmd_Move_Curve(Sub_Chasis sub_Chasis, double setpoint, double radio) {
    //Referencia a variables locales y requerimientos
    this.Chasis = sub_Chasis;
    this.Setpoint = setpoint;
    this.Radio = radio;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Reinicio e inicializacion de variables
    Dt = 0; LastDt = 0; I_Zone = 10;
  RightErrorP = 0; RightErrorI = 0; RightErrorD = 0; RightLastError = 0; RightSpeed = 0;
    LeftErrorP = 0; LeftErrorI = 0; LeftErrorD = 0; LeftLastError = 0; LeftSpeed = 0;
    kP = 0.015; kI = 0.015; 
    kD=0.022;
    kG=0.01;     
    kGI=0.001;
    RightSetpoint=Setpoint*(Math.abs(Radio-12)/Math.abs(Radio));
    LeftSetpoint=Setpoint*(Math.abs(Radio+12)/Math.abs(Radio));
    Chasis.resetPartialEncoder();
    Chasis.resetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Delta de tiempo
    Dt = Timer.getFPGATimestamp() - LastDt;

    //P
    RightErrorP = RightSetpoint - Chasis.getRightEncoder();
    LeftErrorP = LeftSetpoint - Chasis.getLeftEncoder();

    
    //I
    if(RightErrorP <= I_Zone){ RightErrorI += RightErrorP * Dt; }else{ RightErrorI = 0; }
    if(LeftErrorP <= I_Zone){ LeftErrorI += LeftErrorP * Dt; }else{ LeftErrorI = 0; }

    //D
    RightErrorD = (RightErrorP-RightLastError)/Dt;
    LeftErrorD = (LeftErrorP-LeftLastError)/Dt;

    //Giro

    Arco=(Chasis.getRightEncoder()+Chasis.getLeftEncoder())/2;
    ErrorTeta=(180*Arco)/(Radio*Math.PI)-Chasis.getYaw();
    if(ErrorTeta <= 5 && ErrorTeta > 0.5){ ErrorTetaI += ErrorTeta * Dt; }else{ ErrorTetaI = 0; }


    //Corrección de setpoint para cada radio de giro

    DifEnc=(Chasis.getRightEncoder()*(Math.abs(Radio-12)/Math.abs(Radio+12)))-Chasis.getLeftEncoder();

    




    //Control de velocidad
    RightSpeed = (RightErrorP * kP) + (RightErrorI * kI) + (RightErrorD * kD)-(ErrorTeta * kG)-(ErrorTetaI * kGI)-(0.2*DifEnc);
    LeftSpeed = (LeftErrorP * kP) + (LeftErrorI * kI) + (LeftErrorD * kD)+(ErrorTeta * kG)+(ErrorTetaI * kGI)+(0.2*DifEnc);

    //Set a los motores
    Chasis.setSpeed(RightSpeed, LeftSpeed);

    //Retroalimentacion de errores y tiempos
    RightLastError = RightErrorP;
    LeftLastError = LeftErrorP;
    LastDt = Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Control de error al 1%
    if(Math.abs(Arco) == Setpoint){ return true; }else{ return false; }
  }
}

