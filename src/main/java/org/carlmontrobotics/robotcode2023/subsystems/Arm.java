package org.carlmontrobotics.robotcode2023.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.robotcode2023.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Arm extends SubsystemBase
{
    public CANSparkMax motorL = MotorControllerFactory.createSparkMax(Constants.Arm.portL, TemperatureLimit.NEO);
    public CANSparkMax motorR = MotorControllerFactory.createSparkMax(Constants.Arm.portR, TemperatureLimit.NEO);
    public RelativeEncoder motorLencoder = motorL.getEncoder();
    public RelativeEncoder motorRencoder = motorR.getEncoder();

    public double encoderErrorTolerance = .05;

    



    //MotorL is the leader btw
    public Arm(){
    motorR.follow(motorL, true);

    SmartDashboard.putNumber("EncoderStartPosition", 0);
    SmartDashboard.putNumber("EncoderMidPosition", 0.2);
    SmartDashboard.putNumber("EncoderHighPosition", 0.4);

    
    
    }
   
    //Tests if possition in right or wrong
    public String armPosition(){
        
        double EncoderPosition = motorRencoder.getPosition();
    
        if(EncoderPosition<SmartDashboard.getNumber("EncoderStartPosition",-1)+encoderErrorTolerance &&  EncoderPosition>SmartDashboard.getNumber("EncoderStartPosition",-1)-encoderErrorTolerance){
            return "Start";
        } else if(EncoderPosition<SmartDashboard.getNumber("EncoderMidPosition",-1)+encoderErrorTolerance && 
        EncoderPosition>SmartDashboard.getNumber("EncoderMidPosition",-1)-encoderErrorTolerance){
            return "Mid";
        } else if(EncoderPosition<SmartDashboard.getNumber("EncoderHighPosition",-1)+encoderErrorTolerance && 
        EncoderPosition>SmartDashboard.getNumber("EncoderHighPosition",-1)-encoderErrorTolerance){
            return "High";
        } else {
            return "Error";
        }
       
    
    }
}

