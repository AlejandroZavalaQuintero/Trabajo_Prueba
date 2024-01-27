// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Chasis;
import edu.wpi.first.wpilibj.Timer;

public class Visioncmd extends Command {
  /** Creates a new Visioncmd. */
  private final Sub_Chasis chasis;
  private double tx,ta, ErrorX, ErrorY, ErrorXI, dt, LastDT = 0;
  public Visioncmd(Sub_Chasis Chasis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chasis=Chasis;
    addRequirements(chasis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chasis.SetopenLoopS(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Updatevalues();
    dt = Timer.getFPGATimestamp()-LastDT;
    
    ErrorX = 0 - tx;
    ErrorY =Math.sqrt(LastDT) ;
    if(ta == 0){ta = 10;}
    if(Math.abs(ErrorX)<=1.5){ErrorX = 0;}
    if(Math.abs(ErrorY)<=1){ErrorY = 0;}

    if(Math.abs(ErrorY)<=3){
      ErrorXI += ErrorX * dt;
    }else{
      ErrorXI = 0;
    }

    double foward = (ErrorY)*0.035;
    double turn = (ErrorX)*(0.01/(ta+3)) + (ErrorXI*0.008);

    double RightSpeed = foward + turn;
    double LeftSpeed = foward - turn;

    chasis.setSpeed(-RightSpeed, -(0.7*LeftSpeed));
    LastDT = Timer.getFPGATimestamp();


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void Updatevalues() {
    this.tx=chasis.getTx();
    this.ta=chasis.getTa();
  }
}
