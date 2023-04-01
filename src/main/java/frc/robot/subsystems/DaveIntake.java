// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DaveIntake extends SubsystemBase {
  protected final CANSparkMax Neo;
  
  protected final DoubleSolenoid solenoid;
  protected final Compressor compressor;

  /** Creates a new IntakeV1_Lentz. */
  public DaveIntake() {
    Neo = new CANSparkMax(Constants.DAVE_NEO, MotorType.kBrushless);
    //rightNeo = new CANSparkMax(15, MotorType.kBrushless);

    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 15, 14);
    
    compressor = new Compressor(PneumaticsModuleType.REVPH);
  }

  public void setSpeed(double speed) {
    Neo.set(speed);
    
  }
  
  public void setCompressor(boolean isOn){
    if(isOn){
      compressor.enableDigital();
    }
    else{
      compressor.disable();}
    }

  public void setSolenoid(DoubleSolenoid.Value value) {
    solenoid.set(value);
  }
  public DoubleSolenoid.Value getSolenoid(){
    return solenoid.get();
  }
  public boolean hasItem(){
    return Neo.getOutputCurrent()>30;
  }

  @Override
  public void periodic() {
    
    boolean itemIntake = hasItem();
    SmartDashboard.putBoolean("HaveItem", itemIntake);
    SmartDashboard.putNumber("IntakeAmps", Neo.getOutputCurrent());
    SmartDashboard.putBoolean("Running", hasItem());
    // BlinkinLEDMode ledMode = BlinkinLEDMode.BLUE;
    RevBlinkin.set(RevBlinkin.ledMode.getValue());
    
    if (hasItem() == true) {                                
      RevBlinkin.set(RevBlinkin.BlinkinLEDMode.GREEN.getValue());
    }
  }  
}




