// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.fasterxml.jackson.core.json.WriterBasedJsonGenerator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.COTSFalconSwerveConstants.driveGearRatios;
import frc.robot.MiniPID;
import frc.robot.Constants.Swerve;
import pabeles.concurrency.ConcurrencyOps.Reset;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.*;

import java.util.Base64.Decoder;

import javax.swing.JToggleButton.ToggleButtonModel;
import javax.swing.plaf.SliderUI;

import com.ctre.phoenix.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer = new RobotContainer();

  private final Timer sensor_Timer = new Timer();
  private final Timer mode3_Timer = new Timer();
  private final Timer mode_4_5_Timer = new Timer();
  private final Timer mode6_timer = new Timer();
  private final Timer mode7_8_timer = new Timer();

  private final TalonFX m_wristMotor = new TalonFX(15);
  private final TalonFX m_infeedMotor = new TalonFX(16);

  private final MiniPID wristPID = new MiniPID(0, 0, 0);
  private final AnalogPotentiometer wristPot = new AnalogPotentiometer(0);
  private final DigitalInput Sensor = new DigitalInput(0);

  // private final XboxController m_drivController = new XboxController(0);
  private final Joystick m_drivController = new Joystick(0);
  private final Joystick m_flightStick = new Joystick(1);

  Compressor pcmCompressor = new Compressor(50, PneumaticsModuleType.REVPH);
  DoubleSolenoid armDoublePH = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 4, 3);

  private boolean autoEnabled = false;
  // PneumaticsControlModule PCM = new PneumaticsControlModule(50);

  // boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();

  int getMode() {
    return Signaling.mode;
  }

  void setMode(int mode) {
    Signaling.mode = mode;
  }

  int cargo = 0;
  int wristPos = 0;
  double wristPIDOutput = 0;
  int armPos = 0;
 
  
//positions
  double comp = 0.70;

  double coneIn = 0.205;//.22
  double cubeIn = 0.325;//.32
  double coneInShelf = 0.56;
  double cubeInShelf = comp;

  double coneHigh = 0.34;
  double cubeHigh = 0.44;//.44

  double coneMid = 0.22;
  double cubeMid = 0.70; // same as complience


  public int Mode(int mode) {
    switch (mode) {

   //infeeding
      case 1: // infeed cone
      {
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), coneIn));//wristPID.getOutput is a percentage, add a negative to it to inverse the way the motor spins
        // wristPIDOutput = wristPID.getOutput(wristPot.get(), coneIn);
        armDoublePH.set(Value.kReverse);// arm pos
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, -0.75);// motor speed
        if (Sensor.get() && sensor_Timer.hasElapsed(.1)) {
          mode = 3;
        }
        break;
      }

      case 2: // infeed cube
      {  
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeIn));// wrist pos
        // wristPIDOutput = wristPID.getOutput(wristPot.get(), cubeIn);// wrist pos
        armDoublePH.set(Value.kReverse);// arm pos
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0.35);// motor speed
        if (Sensor.get() && sensor_Timer.hasElapsed(.175)) {
          mode = 3;
        }
        break;

      }

      case 21:{
        armDoublePH.set(Value.kReverse);
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), coneInShelf));
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, -0.75);// motor speed
        if (Sensor.get() && sensor_Timer.hasElapsed(.1)) {
          mode = 3;
        }
        break;
      }

      case 22:{
        armDoublePH.set(Value.kReverse);
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeInShelf));
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0.35);// motor speed
        if (Sensor.get() && sensor_Timer.hasElapsed(.175)) {
          mode = 3;
        }
        break;
      }
//#######################################################################################################################
      case 3: // compliance
      {
        // wristPID.reset();
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0);
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), comp));        
       armDoublePH.set(Value.kReverse);// arm pos manual input for complience
        // if (mode3_Timer.hasElapsed(3)) {
          // mode = 0;
        // }
        break;
      }
//#######################################################################################################################

//scoreing
      case 4: // high cone
      {
        // wristPID.reset();
        armDoublePH.set(Value.kForward);
        if (mode_4_5_Timer.hasElapsed(.5)){
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), coneHigh));
      }
      break;
    }
      case 5: // high cube
      {
        
        // wristPID.reset();
        armDoublePH.set(Value.kForward);
        if (mode_4_5_Timer.hasElapsed(0.85)){
          m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeHigh));
        }
          break;        
      }
    
      
      case 6: { // cone mid
        // wristPID.reset();
          armDoublePH.set(Value.kForward);  
          
        // if (mode6_timer.hasElapsed(1)){
          m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), coneMid)); 
        // }       
        break;
      }
      case 9: {
        // wristPID.reset();
          armDoublePH.set(Value.kReverse);     
          m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeMid)); 
          break;
      }


//#########################################################################################################      
      case 7: { //outfeed cone
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, .5); 
        
       break;

      }
      case 8: { //outfeed cube
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, -1);
        
        break;
      }
//################################################################################################################


//modes used for auto
     
      case 12:{// auto comp
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0);
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), 0.70));

        break;
      }
      case 13:{//auto arms up
        armDoublePH.set(Value.kForward);

        break;
      }
      case 14:{//cone wrist up
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(),0.32));
        armDoublePH.set(Value.kForward);

        break;
      }

      case 15:{//cube wrist up
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(),0.44));
        armDoublePH.set(Value.kForward);

        break;
      }
      // case 16: {
      //   m_infeedMotor.set(TalonFXControlMode.PercentOutput, .4); 
      //     break;
      // }

      // case 18: {
      //   m_infeedMotor.set(TalonFXControlMode.PercentOutput, -2); 
      //     break;
      // }

      case 17:{
      //  wristPID.reset();
          m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeIn));// wrist pos
          // wristPIDOutput = wristPID.getOutput(wristPot.get(), cubeIn);// wrist pos
  
          armDoublePH.set(Value.kReverse);// arm pos
  
          m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0.5);// motor speed
          if (Sensor.get()) {// sensor delay
            sensor_Timer.start();
          } else {
            sensor_Timer.reset();}

          if (Sensor.get() && sensor_Timer.hasElapsed(.3 )) {
            mode = 3;
          }
          break; 
      }

      // case 19:{
      //   m_infeedMotor.set(TalonFXControlMode.PercentOutput, -0.5);// motor speed

      // }

      // case 20:{
      //   armDoublePH.set(Value.kReverse);// arm pos manual input for complience

      // }

//#################################################################################################################

//auto balance?
     
//later

    }
    return mode;
    }
   


  public void setIntakePneumatics(boolean out) {
    if (out) {
      wristPID.setSetpoint(0.12);
    }
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    wristPID.setP(3.00125);
    wristPID.setI(0.933510000001);
    wristPID.setD(12);
    // wristPID.setF(0); //dont toutch unless you know what it will do
    wristPID.setOutputLimits(-.6, .6);//.45
    wristPID.setOutputRampRate(0.07570); 
    wristPID.setMaxIOutput(1.6);
    
  }

  /**
   * This function is call.ed every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    //if(MathUtill.abs(currentPos-Setpoint)<5){ //if error less then 5
      //do nothint
   // }else{
     wristPID.reset();
   // }

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    autoEnabled = false;
  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autoEnabled = true;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    wristPID.reset();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // The code to control the arm and wrist movement can be put here.
    runLoop();

  }

  @Override
  public void teleopInit() {
    autoEnabled = false;
    wristPID.reset();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

     pcmCompressor.enableAnalog(0, 120);
      armDoublePH.set(Value.kReverse);
      setMode(0);
      
      sensor_Timer.reset();
      sensor_Timer.start();

      mode3_Timer.reset();
      mode3_Timer.start();

      mode_4_5_Timer.reset();
      mode_4_5_Timer.start();

      mode6_timer.reset();
      mode6_timer.start();

      mode7_8_timer.start();
      mode7_8_timer.start();

      
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    runLoop();

  }

  private void runLoop() {
    wristPID.reset();
    // if(m_drivController.getRawButtonPressed(2)){
    // wristPos = 1 - wristPos;
    // }
    // else if (m_drivController.getRawButtonPressed(2) && wristPos >= 1){
    // wristPos--;
    // }

    // for pid testing (wrist PID)
    // if(wristPos == 0){
    // m_wristMotor.set(TalonFXControlMode.PercentOutput ,
    // -wristPID.getOutput(wristPot.get(), .6));
    // wristPIDOutput = wristPID.getOutput(wristPot.get(), 0.6);
    // }
    // else{
    // m_wristMotor.set(TalonFXControlMode.PercentOutput ,
    // -wristPID.getOutput(wristPot.get(), 0.511));
    // wristPIDOutput = wristPID.getOutput(wristPot.get(), 0.511);

    // }

    // wrist
    if (m_flightStick.getRawButton(3))
      m_wristMotor.set(TalonFXControlMode.PercentOutput, .25);
    else if (m_flightStick.getRawButton(4))
      m_wristMotor.set(TalonFXControlMode.PercentOutput, -.25);
    else
      m_wristMotor.set(TalonFXControlMode.PercentOutput, 0);



    SmartDashboard.putNumber("wristPot", wristPot.get());
    SmartDashboard.putNumber("flight stick axis", m_flightStick.getRawAxis(1));
    SmartDashboard.putNumber("wrist POS", wristPos);
    SmartDashboard.putNumber("wristMotorPID output", wristPIDOutput);
    SmartDashboard.putNumber("mode", getMode());
    // SmartDashboard.putNumber("memory_mode", memory_mode);
    SmartDashboard.putBoolean("compressor", pcmCompressor.isEnabled());
    SmartDashboard.putBoolean("Auto Enabled", autoEnabled);
    SmartDashboard.putNumber("airPressure", pcmCompressor.getPressure());
    SmartDashboard.putNumber("WristPid", wristPIDOutput);
    SmartDashboard.putBoolean("sensor", Sensor.get());
    SmartDashboard.putNumber("Wrist I", wristPID.getOutput());


    // SmartDashboard.putNumber("swerveAngle");



    // else {m_infeedMotor.set(0);}
    if (!autoEnabled) {
     
      if (m_drivController.getRawButtonPressed(5)) {// panic button  (9 for xbox, 5 for joystick)
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0);
        setMode(0);
      }
      if (m_flightStick.getRawButtonPressed(1)){
        setMode(0);
      }
     
//#################################################################################################

      //infeeding
      if (m_drivController.getRawButtonPressed(7) && (getMode() != 4 || getMode() != 5)) {// infeed cone (5 for xbox, 7 for joystick)                                                                            
        setMode(1);
        wristPID.reset();
        sensor_Timer.reset();
        sensor_Timer.start();
        }

      if (m_drivController.getRawButtonPressed(6) && (getMode() != 4 || getMode() != 5)) { // infeed cube   (6 for both)                                                                       
        setMode(2);
        wristPID.reset();
        sensor_Timer.reset();
        sensor_Timer.start();
      }
      if (m_drivController.getRawButtonPressed(16) && (getMode() !=4 || getMode() != 5)){
        setMode(21);
      }
      if (m_drivController.getRawButtonPressed(15) && (getMode() !=4 || getMode() != 5)){
        setMode(22);
      }

//#################################################################################################

      // compliance
     if (m_drivController.getRawButtonPressed(2)) {// complience   (10 for xbox, 2 for joyatick)   
        setMode(3);
        wristPID.reset();
      }
      
//########################################################################################

  //scoring

      //real
      if (m_drivController.getRawButtonPressed(3)) {//cone high (1 for xbox, 3 for joystick)
        setMode(4);
        wristPID.reset();
        mode_4_5_Timer.reset();
        mode_4_5_Timer.start();
      }      
      if (m_drivController.getRawButtonPressed(4)) {//cube high (2 for xbox, 4 for joystick)
        setMode(5);
        wristPID.reset();
        mode_4_5_Timer.reset();
        mode_4_5_Timer.start();
      }
      if (m_drivController.getRawButton(10)) { // cone mid (7 for xbox, 10 for joystick)
        setMode(6);
        wristPID.reset();
      }


      if(m_drivController.getRawButton(1) && (getMode() == 4 || getMode() == 6) && (getMode() != 5) ) {// outfeed cone  (3 for xbox, 1 for joystick)
        setMode(7);

      } else if(m_drivController.getRawButton(1) && ((getMode() == 5) || (getMode() == 3)) && (getMode() != 4)) {// outfeed cube  (3 for xbox, 1 for joystick)                                                                             
        setMode(8);
      }
      if (m_drivController.getRawButton(9) && (getMode() == 3)){ //low cube
        setMode(19);

      }
      if (m_drivController.getRawButton(14)){// low cone (14 for joustick  7 for xbox )
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 2);
      }
      
//##############################################################################################


//these next few are what starts and resets the timers used in the modes
      if (Sensor.get()) {// sensor delay
        sensor_Timer.start();
      } else {
        sensor_Timer.reset();}

      // if (getMode() == 3) {// mode 3 timer
      //   mode3_Timer.start();
      // } else {
      //   mode3_Timer.reset();
      // }


      // if (getMode() == 4 || getMode() == 5){
      //   mode_4_5_Timer.start();
      // }
      //  else{
      //   mode_4_5_Timer.reset();
      //  }


      // if ((getMode() == 6)) { //guess what its for
      //   mode6_timer.start();
      // } else {
      //   mode6_timer.reset();
      // }
      // if (getMode() == 7  || getMode() == 8){ 
      //   mode7_8_timer.start();
      // }
      // else {
      //   mode7_8_timer.reset();
      // }

      
      



//#####################################################################################


      // pneumatic commands

      if (m_flightStick.getRawButtonPressed(2)) { // enable compressor
        pcmCompressor.enableAnalog(100, 120);
      } 
      

      

      if (m_flightStick.getRawButtonPressed(5)) {
        armDoublePH.set(Value.kForward);
      }
      if (m_flightStick.getRawButtonPressed(6)) {
        armDoublePH.set(Value.kReverse);
      }

      
    }

    setMode(Mode(getMode()));
  }// important
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}


