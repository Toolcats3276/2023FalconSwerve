// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.COTSFalconSwerveConstants.driveGearRatios;
import frc.robot.MiniPID;
import pabeles.concurrency.ConcurrencyOps.Reset;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.*;
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
  double armPIDOutput = 0;
  int memory_mode = 0;
  
//positions
  double comp = 0.70;

  double coneIn = 0.20;
  double cubeIn = 0.32;

  double coneHigh = 0.34;
  double cubeHigh = 0.44;

  double coneMid = 0.22;
  double cubeMid = 0.70; // same as complience


  public int Mode(int mode) {
    switch (mode) {

   //infeeding
      case 1: // infeed cone
      {
        memory_mode = 1;
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), coneIn));
        // wristPIDOutput = wristPID.getOutput(wristPot.get(), coneIn);

        armDoublePH.set(Value.kReverse);// arm pos

        m_infeedMotor.set(TalonFXControlMode.PercentOutput, -0.75);// motor speed

        if (Sensor.get() && sensor_Timer.hasElapsed(.15)) {
          mode = 3;
          break;
        }

        break;

      }

      case 2: // infeed cube
      {
        memory_mode = 2;
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeIn));// wrist pos
        // wristPIDOutput = wristPID.getOutput(wristPot.get(), cubeIn);// wrist pos

        armDoublePH.set(Value.kReverse);// arm pos

        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0.5);// motor speed

        if (Sensor.get() && sensor_Timer.hasElapsed(.3 )) {
          mode = 3;
          break;
        }
        break;

      }
//#######################################################################################################################
      case 3: // transporting cargo
      {
      
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
        armDoublePH.set(Value.kForward);
        if (mode_4_5_Timer.hasElapsed(1)){
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), coneHigh));}
        
       break;
      }

      case 5: // high cube
      {
        armDoublePH.set(Value.kForward);
        if (mode_4_5_Timer.hasElapsed(0.85)){
          m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeHigh));
        }
          break;        
        }
     
      
      case 6: { // cone mid
          armDoublePH.set(Value.kForward);  
          
        if (mode6_timer.hasElapsed(1)){
          m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), coneMid)); 
        }       
        break;
      }
      case 9: {
          armDoublePH.set(Value.kReverse);     
          m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeMid)); 
          break;
      }


//#########################################################################################################      
      case 7: { //outfeed cone
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, .5); 
        memory_mode = 0;
       break;

      }
      case 8: { //outfeed cube
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, -2);
        memory_mode = 0;
        break;
      }
//################################################################################################################


//modes used for auto
      case 11:{ //auto outfeed
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, -1.5);

        break;
      }
      case 12:{// auto comp
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0);
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), comp));
        armDoublePH.set(Value.kReverse);// arm pos manual input for complience

        break;
      }
      case 13:{//auto arms up
        armDoublePH.set(Value.kForward);

        break;
      }
      case 14:{//auto wrist up
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(),cubeHigh));
        armDoublePH.set(Value.kForward);

        break;
      }
      
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
    wristPID.setP(6.5);
    wristPID.setI(0.0005);
    wristPID.setD(1.2);
    wristPID.setOutputLimits(-.75, .75);
    wristPID.setOutputRampRate(2);
    // wristPID.setMaxIOutput(kDefaultPeriod);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
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
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    wristPID.reset();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

      pcmCompressor.enableAnalog(0, 120);
      armDoublePH.set(Value.kReverse);
      setMode(0);
      memory_mode = 0;
      sensor_Timer.reset();
      mode3_Timer.reset();
      mode_4_5_Timer.reset();
      mode6_timer.reset();
      mode7_8_timer.start();
      
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    runLoop();

  }

  private void runLoop() {
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
      m_wristMotor.set(TalonFXControlMode.PercentOutput, .5);
    else if (m_flightStick.getRawButton(4))
      m_wristMotor.set(TalonFXControlMode.PercentOutput, -.5);
    else
      m_wristMotor.set(TalonFXControlMode.PercentOutput, 0);



    SmartDashboard.putNumber("wristPot", wristPot.get());
    SmartDashboard.putNumber("flight stick axis", m_flightStick.getRawAxis(1));
    SmartDashboard.putNumber("wrist POS", wristPos);
    SmartDashboard.putNumber("wristMotorPID output", wristPIDOutput);
    SmartDashboard.putNumber("mode", getMode());
    SmartDashboard.putNumber("memory_mode", memory_mode);
    SmartDashboard.putBoolean("compressor", pcmCompressor.isEnabled());
    SmartDashboard.putBoolean("Auto Enabled", autoEnabled);
    SmartDashboard.putNumber("airPressure", pcmCompressor.getPressure());




    // else {m_infeedMotor.set(0);}
    if (!autoEnabled) {
     
      if (m_drivController.getRawButtonPressed(5)) {// panic button  (7 for xbox)
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0);
        setMode(0);
      }
      if (m_flightStick.getRawButtonPressed(1)){
        setMode(0);
      }
     
//#################################################################################################

      //infeeding
      if (m_drivController.getRawButtonPressed(7) && (getMode() != 4 || getMode() != 5)) {// infeed cone (5 for xbox)
                                                                                          
        setMode(1);
        }

      if (m_drivController.getRawButtonPressed(6) && (getMode() != 5 || getMode() != 4)) { // infeed cube   (6 for xbox)
                                                                                           
        setMode(2);
      }

      //test with memory mode
      // if (m_drivController.getRawButtonPressed(3) && (getMode() != 4 || getMode() != 5 || getMode() != 3)) {
      //   setMode(1);
      // }
      // if (m_drivController.getRawButtonPressed(4) && (getMode() != 4 || getMode() != 5 || getMode() != 3)) {
      //   setMode(2);
      // }
//#################################################################################################

      // compliance
     if (m_drivController.getRawButtonPressed(2)) {// complience   (10 for xbox)   
        setMode(3);
      }
      
//########################################################################################

  //scoring

      //test
      // if (m_drivController.getRawButtonPressed(3) && (memory_mode == 1) && (getMode() == 3) && (getMode() != 5)) {//cone high  (3 for xbox)
      //   setMode(4);
      // }
      // if (m_drivController.getRawButtonPressed(4) && (memory_mode == 2) && (getMode() == 3) && (getMode() != 4)) {//cube high (1 for xbox)
      //   setMode(5);
      // }


      //real
      if (m_drivController.getRawButtonPressed(3)) {//cube high (1 for xbox)
        setMode(4);
      }
      if (m_drivController.getRawButtonPressed(4)) {//cube high (1 for xbox)
        setMode(5);
      }



      if (m_drivController.getRawButton(10)) { // cone mid (9 for xbox)
        setMode(6);
      }


      if (m_drivController.getRawButton(1) && (getMode() == 4 || getMode() == 6) && (getMode() != 5) ) {// outfeed cone  (2 for xbox)
        setMode(7);

      } else if (m_drivController.getRawButton(1) && ((getMode() == 5) || (getMode() == 3)) && (getMode() != 4)) {// outfeed cube  (2 for xbox)
                                                                                                   
        setMode(8);
      }
      if (m_drivController.getRawButton(9)){ //mid cube
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 2);


      }
      if (m_drivController.getRawButton(14)){// low cone
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, -2);
      }
      if (m_drivController.getRawButtonPressed(2)){
        wristPID.reset();
      }
//##############################################################################################


//these next few are what starts and resets the timers used in the modes
      if (Sensor.get()) {// sensor delay
        sensor_Timer.start();
      } else {
        sensor_Timer.reset();}

      if (getMode() == 3) {// mode 3 timer
        mode3_Timer.start();
      } else {
        mode3_Timer.reset();
      }


      if (getMode() == 4 || getMode() == 5){
        mode_4_5_Timer.start();
      }
       else{
        mode_4_5_Timer.reset();
       }


      if ((getMode() == 6)) { //guess what its for
        mode6_timer.start();
      } else {
        mode6_timer.reset();
      }
      if (getMode() == 7  || getMode() == 8){ 
        mode7_8_timer.start();
      }
      else {
        mode7_8_timer.reset();
      }

      
      



//#####################################################################################


      // pneumatic commands

      if (m_flightStick.getRawButton(8)) { // enable compressor
        pcmCompressor.enableAnalog(100, 120);
      } else if (m_flightStick.getRawButton(9)) {
        pcmCompressor.disable();
      }

      // armDoublePH.set(Value.kForward);

      if (m_flightStick.getRawButtonPressed(5)) {
        armDoublePH.set(Value.kForward);
      }
      if (m_flightStick.getRawButtonPressed(6)) {
        armDoublePH.set(Value.kReverse);
      }

      
    }

    setMode(Mode(getMode()));// important
  }

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
