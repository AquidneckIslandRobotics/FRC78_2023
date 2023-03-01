// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Dave_Intake extends SubsystemBase {
  protected final CANSparkMax Neo;
  
  protected final DoubleSolenoid dropSolenoid;
  protected final DoubleSolenoid pieceSolenoid;

  protected final Compressor compressor;

  /** Creates a new IntakeV1_Lentz. */
  public Dave_Intake() {
    Neo = new CANSparkMax(Constants.DAVE_NEO, MotorType.kBrushless);
    //rightNeo = new CANSparkMax(15, MotorType.kBrushless);

    dropSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 15, 14);
    pieceSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 13, 12);//subject to change 
    
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

  public void setSolenoidDrop(DoubleSolenoid.Value value) {
    dropSolenoid.set(value);
  }

  public void setSolenoidPiece(DoubleSolenoid.Value value) {
    pieceSolenoid.set(value);

}
  public DoubleSolenoid.Value getDropSolenoid(){
    return dropSolenoid.get();
  }

  public DoubleSolenoid.Value getPieceSolenoid() {
    return pieceSolenoid.get();
  }

  public boolean hasItem(){
    return Neo.getOutputCurrent()>30;
  }

  @Override
  public void periodic() {
    boolean itemIntake = hasItem();
    SmartDashboard.putBoolean("HaveItem", itemIntake);
    SmartDashboard.putNumber("IntakeAmps", Neo.getOutputCurrent());
  }  
}




