package frc.robot.subsystems;


import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NavX.AHRS;
import edu.wpi.first.networktables.*;



public class Sub_Chasis extends SubsystemBase {
  private double Volts;
  private double PartialEncoderL;
  private double PartialEncoderR;
  private double DifEncoderL;
  private double DifEncoderR;
  private double DifYaw;
  private double PartialYaw;




  //Build motores
  CANSparkMax MotorDM = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax MotorDS = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax MotorIM = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax MotorIS = new CANSparkMax(4, MotorType.kBrushless);
 // CANSparkMax MotorIntake = new CANSparkMax(10, MotorType.kBrushless);

  //Encoders
  private final RelativeEncoder EncoderR = MotorDM.getEncoder();
  private final RelativeEncoder EncoderL = MotorIM.getEncoder();
  //gyroscopio
  AHRS ahrs = new AHRS(SPI.Port.kMXP, (byte)66);
  //Control
  CommandXboxController JoyDrive = new CommandXboxController(0);
  public double Tx, Ty, Ta;
  public double Dt,LastDt;
  public double AcceX,VelX,DisX;
  public double AcceY,VelY,DisY, origen;


  public Sub_Chasis() {
    
    PartialEncoderL=0;
    PartialEncoderR=0;
    PartialYaw=0;
    LastDt=0;
    

    MotorDM.restoreFactoryDefaults();
    MotorDS.restoreFactoryDefaults();
    MotorIM.restoreFactoryDefaults();
    MotorIS.restoreFactoryDefaults();
    //MotorIntake.restoreFactoryDefaults();

    MotorDM.setIdleMode(IdleMode.kBrake);
    MotorDM.set(0);
    MotorDS.setIdleMode(IdleMode.kBrake);
    MotorDS.set(0);
    MotorIM.setIdleMode(IdleMode.kBrake);
    MotorIM.set(0);
    MotorIS.setIdleMode(IdleMode.kBrake);
    MotorIS.set(0);
    /* 

    MotorIntake.setIdleMode(IdleMode.kCoast);
    MotorIntake.set(0);
*/
    MotorDS.follow(MotorDM);
    MotorIS.follow(MotorIM);

    MotorIM.setInverted(true);

    EncoderL.setPosition(0);
    EncoderR.setPosition(0);

    EncoderR.setPositionConversionFactor(2.02);
    EncoderL.setPositionConversionFactor(2.02);

    Volts=RobotController.getBatteryVoltage();
    


  }

  @Override
  public void periodic() { 
    Dt = Timer.getFPGATimestamp() - LastDt;
    //SmartDashboard.putNumber("Velocidad Chasis", avgSpeed());
    /*SmartDashboard.putNumber("encoderizquierdo", getLeftEncoder());
    SmartDashboard.putNumber("encoderderecho", getRightEncoder());
    SmartDashboard.putNumber("RightSpeed", MotorDM.get());
    SmartDashboard.putNumber("LefttSpeed", MotorIM.get());
    SmartDashboard.putNumber("Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("Roll", ahrs.getRoll());*/
    
    AcceX=ahrs.getWorldLinearAccelX();
    VelX+=AcceX*Dt;
    
    DisY=getTotalYaw();
    DisX=(ahrs.getDisplacementX())*(16/0.6753);
    LastDt = Timer.getFPGATimestamp();
  
    SmartDashboard.putNumber("Distancia X",DisX);
    SmartDashboard.putNumber("Distancia aproximada en progra",DisY);
    //MotorIntake.set(0);
    
  }
  /* 
  private double avgSpeed(){
    double avgSpeed = (RightEncoder.getVelocity() + LeftEncoder.getVelocity()) / 2; return avgSpeed;
  } */

  public void SetopenLoopS(double S){
    MotorDM.setOpenLoopRampRate(S);
    MotorIM.setOpenLoopRampRate(S);
    MotorDS.setOpenLoopRampRate(S);
    MotorIS.setOpenLoopRampRate(S);
  }
  
  public void resetTotalEncoder(){
    EncoderL.setPosition(0);
    EncoderR.setPosition(0);
  }

  public void resetPartialEncoder(){
    PartialEncoderL=EncoderL.getPosition();
    PartialEncoderR=EncoderR.getPosition();  
  }

  public double getLeftTotalEncoder(){
    return EncoderL.getPosition();
  }

  public double getRightTotalEncoder(){
    return EncoderR.getPosition();
  }

  public double getLeftEncoder(){
    DifEncoderL=(EncoderL.getPosition())-PartialEncoderL;
    return DifEncoderL;
  }

  public double getRightEncoder(){
    DifEncoderR=(EncoderR.getPosition())-PartialEncoderR;
    return DifEncoderR;
  }
  public void resetTotalYaw(){
    ahrs.reset();
  }  
  public void resetYaw(){
    PartialYaw=ahrs.getYaw();
  }
  public double getTotalYaw(){
    return ahrs.getYaw();
  }
  public double getYaw(){
    DifYaw=(ahrs.getYaw())-PartialYaw;
    return DifYaw;
  }
  public double getpromencoders(){
    return (getRightEncoder()+getLeftEncoder())/2;
  }
  public void setSpeed(double RightSpeed,double LeftSpeed){
    if(Math.abs(LeftSpeed) >= 0.6){LeftSpeed = (LeftSpeed/Math.abs(LeftSpeed))*0.6;}
    if(Math.abs(RightSpeed) >= 0.6){RightSpeed = (RightSpeed/Math.abs(RightSpeed))*0.6;}
    MotorDM.set(RightSpeed);
    MotorIM.set(LeftSpeed);
  }
  public void IntakeSpeed(double IntakeSpeed){
    //MotorIntake.set(IntakeSpeed);
  }
  public double getPitch(){
    return ahrs.getPitch();
  }
  public double getRoll(){
    return ahrs.getRoll();
  }
  public double getP(){
    return 0.01;
  }
  public double getI(){
    return 0;
  }
  public double getD(){
    return 0;
  }
  public double getTx(){
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("tx").getDouble(0);
  }

  public double getTy(){
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("ta").getDouble(0);
  }

  public double getTa(){
    return NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("ta").getDouble(10);
  }
  
  public void SetVisionMode (Double m){
    NetworkTableInstance.getDefault().getTable("limelight-abtomat").getEntry("pipeline").setNumber(m);
  }
  



}
