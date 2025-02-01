// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

 
public class DriveSubsystem extends SubsystemBase {
  
     public static DifferentialDrive m_drive;
     public double direction = 1.0;
     public double speed_changer = 0.6;

    public WPI_TalonSRX m_MotorRight = new WPI_TalonSRX(2);
    WPI_TalonSRX m_MotorRightFollow = new WPI_TalonSRX(4);
    WPI_TalonSRX m_MotorLeft = new WPI_TalonSRX(3);
    WPI_TalonSRX m_MotorLeftFollow = new WPI_TalonSRX(1);
    public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    public static DutyCycleEncoder encoderDriveR = new DutyCycleEncoder(1);
    public static DutyCycleEncoder encoderDriveL = new DutyCycleEncoder(0);

    private static double lastPositionR;
    private static double lastPositionL;
    private static double lastTime;
            
    static  int kEncoderCPR = 8192;
    static double kWheelDiameterMeters = Units.inchesToMeters(6);
    static double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / kEncoderCPR;
                    
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
                    
    // Create a new SysId routine for characterizing the drive.
    private final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
    // Tell SysId how to plumb the driving voltage to the motors.
            voltage -> {
                m_MotorRight.setVoltage(voltage);
                m_MotorLeft.setVoltage(voltage);
                },
    // Tell SysId how to record a frame of data for each motor on the mechanism being
    // characterized.
        log -> {
    // Record a frame for the left motors.  Since these share an encoder, we consider
    // the entire group to be one motor.
        log.motor("drive-left")
            .voltage(
                m_appliedVoltage.mut_replace(
                -m_MotorLeft.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(encoderDriveL.get(), Meters))
                  .linearVelocity(m_velocity.mut_replace(DriveSubsystem.getLinearVelocityL(),MetersPerSecond));
// Record a frame for the right motors.  Since these share an encoder, we consider
// the entire group to be one motor.
        log.motor("drive-right")
            .voltage(
                m_appliedVoltage.mut_replace(
                -m_MotorRight.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(encoderDriveR.get(), Meters))
                    .linearVelocity(m_velocity.mut_replace(DriveSubsystem.getLinearVelocityR(), MetersPerSecond));
        },
// Tell SysId to make generated commands require this subsystem, suffix test state in
// WPILog with this subsystem's name ("drive")
  this));
                                            
                                                          
                                                          
public DriveSubsystem() {
                                              
                                                
  m_MotorRight.setInverted(true);
  m_MotorRightFollow.setInverted(true);
  m_MotorLeft.setInverted(false);
  m_MotorLeftFollow.setInverted(false);
                                            
  m_MotorRightFollow.follow(m_MotorRight);
  m_MotorLeftFollow.follow(m_MotorLeft);
                                           
  m_MotorRight.configVoltageCompSaturation(11.0);
  m_MotorRightFollow.configVoltageCompSaturation(11.0);
  m_MotorLeft.configVoltageCompSaturation(11.0);
  m_MotorLeftFollow.configVoltageCompSaturation(11.0);
                                              
  m_MotorRight.setSafetyEnabled(true);
  m_MotorRightFollow.setSafetyEnabled(true);
  m_MotorLeft.setSafetyEnabled(true);
  m_MotorLeftFollow.setSafetyEnabled(true);
                                            
                                                
                                                
                                                
  m_drive = new DifferentialDrive(m_MotorLeft,m_MotorRight);
                                            
  lastPositionR = encoderDriveR.get();
  lastPositionL = encoderDriveL.get();
  lastTime = Timer.getFPGATimestamp();
                                            
  int kEncoderCPR = 8192;
  double kWheelDiameterMeters = Units.inchesToMeters(6);
  double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / kEncoderCPR;
                                            
                                                  
                                            
}
                                            
  public static double getLinearVelocityR() {
  // Récupérer la position actuelle
  double currentPositionR = -encoderDriveR.get();
  double currentTime = Timer.getFPGATimestamp();
                                           
  // Calculer la variation de position
  double deltaPosition = currentPositionR - lastPositionR;
                                            
  // Gérer le passage de la position absolue (0 -> 1) à la distance réelle
  if (deltaPosition < -0.5) {
  // Cas où l'encodeur est passé de 1 à 0 (rotation arrière)
  deltaPosition += 1.0;
  } else if (deltaPosition > 0.5) {
  // Cas où l'encodeur est passé de 0 à 1 (rotation avant)
  deltaPosition -= 1.0;
  }
                                            
  double deltaDistance = deltaPosition * kEncoderDistancePerPulse;
  double deltaTime = currentTime - lastTime;
                                            
  // Mettre à jour les valeurs précédentes
  lastPositionR = currentPositionR;
  lastTime = currentTime;
                                            
  // Calculer et retourner la vélocité (distance par seconde)
  return deltaDistance / deltaTime;
  }
                                              
  public static double getLinearVelocityL() {
  // Récupérer la position actuelle
  double currentPositionL = -encoderDriveL.get();
  double currentTime = Timer.getFPGATimestamp();
                    
  // Calculer la variation de position
  double deltaPosition = currentPositionL - lastPositionR;
                
  // Gérer le passage de la position absolue (0 -> 1) à la distance réelle
  if (deltaPosition < -0.5) {
  // Cas où l'encodeur est passé de 1 à 0 (rotation arrière)
    deltaPosition += 1.0;
  } else if (deltaPosition > 0.5) {
  // Cas où l'encodeur est passé de 0 à 1 (rotation avant)
     deltaPosition -= 1.0;
  }
                
  double deltaDistance = deltaPosition * kEncoderDistancePerPulse;
  double deltaTime = currentTime - lastTime;

    // Mettre à jour les valeurs précédentes
    lastPositionL = currentPositionL;
    lastTime = currentTime;

    // Calculer et retourner la vélocité (distance par seconde)
    return deltaDistance / deltaTime;
}

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }


  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
  }

    /*public void arcadeDrive(double fwd, double rot) {
      m_drive.arcadeDrive(fwd*direction, rot);
      m_drive.setMaxOutput(speed_changer);
       m_MotorRightFollow.follow(m_MotorRight);
      m_MotorLeftFollow.follow(m_MotorLeft);
    }*/
    
    public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

    public void tankDrive(double left, double right){
      m_drive.tankDrive(left, right);
    }

    public void setMaxOutput(double speed_changer) {
      m_drive.setMaxOutput(speed_changer);
    }

    public void reverse(){
      direction = -direction;
    }

    public void speedUp(){
      if(speed_changer <= 0.8){
      speed_changer = speed_changer + 0.3;
      }
    }
    public void speedDown(){
      if(speed_changer >= 0.1){
      speed_changer = speed_changer - 0.3;
      }
    }
    public double getAngle(){
      return gyro.getAngle();
    }
    public double getRate(){
      return gyro.getRate();
    }
    public void resetGyro(){
      gyro.reset();
    } 
    public void calibrateGyro(){
      gyro.calibrate();
    }
    public double getRightPosition(){
      return encoderDriveR.get();
    }
    
    public double getLeftPosition(){
      return encoderDriveL.get();
    }
    
   
   
    
  @Override
  public void periodic() {} // This method will be called once per scheduler run

  public void setDriveMotors(double forward, double turn){

    double left = forward - turn;
    double right = forward + turn;

    m_MotorRight.set(TalonSRXControlMode.PercentOutput, right);
    m_MotorLeft.set(TalonSRXControlMode.PercentOutput, left);

    m_MotorRightFollow.set(TalonSRXControlMode.PercentOutput, right);
    m_MotorLeftFollow.set(TalonSRXControlMode.PercentOutput, left);
  }
  public void stop(){
      m_MotorRight.set(0.0);
      m_MotorRightFollow.set(0.0);
      m_MotorLeft.set(0.0);
      m_MotorLeftFollow.set(0.0);
  }
  
  public void drive(double leftPercentPower, double rightPercentPower) {
    m_MotorLeft.set(leftPercentPower);
    m_MotorLeftFollow.set(leftPercentPower);
    m_MotorRight.set(rightPercentPower);
    m_MotorRightFollow.set(rightPercentPower);
  }



  public void driveRight(double speed){
    m_MotorRight.set(speed);

  }
  public void driveLeft(double speed){
    m_MotorLeft.set(speed);
    
  }
  public void driveRightFollow(double speed){
    m_MotorRightFollow.set(speed);
    
  }
  public void driveLeftFollow(double speed){
    m_MotorLeftFollow.set(speed);
    
  }


  

}
