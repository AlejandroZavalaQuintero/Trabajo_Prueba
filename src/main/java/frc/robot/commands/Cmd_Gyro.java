// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.opencv.core.Mat;
import frc.robot.subsystems.Sub_Chasis;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Cmd_Gyro extends Command {
  private final Sub_Chasis Chasis;
  private double Setpoint, Dt, LastDt, I_Zone, Angle;
  private double ErrorP, ErrorI, ErrorD, LastError, RightSpeed;
  private double LeftSpeed;
  private double kP, kI, kD;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Cmd_Gyro(Sub_Chasis sub_Chasis, double setpoint) {
    this.Chasis = sub_Chasis;
    this.Setpoint = setpoint;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Reinicio e inicializacion de variables
    Dt = 0; LastDt = 0; I_Zone = Setpoint*.10;
    ErrorP = 0; ErrorI = 0; ErrorD = 0; LastError = 0; RightSpeed = 0;
    LeftSpeed = 0;
    kP = 0.015; kI = 0.015; kD = 0.0022;
    Angle=Setpoint%360;    
    Chasis.resetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Delta de tiempo
    Dt = Timer.getFPGATimestamp() - LastDt;

    //P
    ErrorP = Angle - Chasis.getYaw();
  

    
    //I
    if(ErrorP <= I_Zone){ ErrorI += ErrorP * Dt; }else{ ErrorI = 0; }

    //D
    ErrorD = (ErrorP-LastError)/Dt;

    //Control de velocidad
    RightSpeed = (ErrorP * kP) + (ErrorI * kI) + (ErrorD * kD);
    LeftSpeed = -RightSpeed;

    //Set a los motores
    Chasis.setSpeed(RightSpeed, LeftSpeed);

    //Retroalimentacion de errores y tiempos
    LastError = ErrorP;
    LastDt = Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Control de error al 1%
    if(Math.abs(Chasis.getYaw()) == Angle){ return true; }else{ return false; }
  }
}
