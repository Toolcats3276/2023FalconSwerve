// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;


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


  MiniPID wristPID = new MiniPID(0, 0, 0);
  private final AnalogPotentiometer wristPot = new AnalogPotentiometer(0);
  private final DigitalInput Sensor = new DigitalInput(0);
  

  private final Joystick m_drivController = new Joystick(0);
  private final Joystick m_flightStick = new Joystick(1);

  Compressor pcmCompressor = new Compressor(50, PneumaticsModuleType.REVPH);
  DoubleSolenoid armDoublePH = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 4, 3);
  DoubleSolenoid slideDoublePH = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 7, 6);

  private boolean autoEnabled = false;

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
  double comp = 0.88;

  double coneIn = 0.33;
  double cubeIn = 0.46;
  // double coneInShelf = 0.56;
  // double cubeInShelf = comp;

  double coneHigh = 0.41;
  double cubeHigh = 0.54;
  //.73 for shooting high cube

  double coneMid = 0.28;
  double cubeMid = comp ; // same as complience

 

  public int Mode(int mode) {
    switch (mode) {

   //infeeding
      case 1: // infeed cone
      {
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), coneIn));//wristPID.getOutput is a percentage, add a negative to it to inverse the way the motor spins
        armDoublePH.set(Value.kReverse);
        slideDoublePH.set(Value.kReverse);

         m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0.75);
        if (Sensor.get() && sensor_Timer.hasElapsed(.4)) {
          mode = 3;
        }
        break;
      }

      case 2: // infeed cube
      {  
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeIn));
        armDoublePH.set(Value.kReverse);
        slideDoublePH.set(Value.kReverse);

         m_infeedMotor.set(TalonFXControlMode.PercentOutput, -0.7);
        if (Sensor.get() && sensor_Timer.hasElapsed(.35)) {
          mode = 3;
        }
        break;

      }

      // case 21:{
      //   armDoublePH.set(Value.kReverse);
      //   slideDoublePH.set(Value.kReverse);
      //   m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), coneInShelf));
      //   m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0.75);// motor speed
      //   if (Sensor.get() && sensor_Timer.hasElapsed(.1)) {
      //     mode = 3;
      //   }
      //   break;
      // }

      // case 22:{
      //   armDoublePH.set(Value.kReverse);
      //   slideDoublePH.set(Value.kReverse);
      //   m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeInShelf));
      //   m_infeedMotor.set(TalonFXControlMode.PercentOutput, -0.35);// motor speed
      //   if (Sensor.get() && sensor_Timer.hasElapsed(.175)) {
      //     mode = 3;
      //   }
      //   break;
      // }
//#######################################################################################################################
      case 3: // compliance
      {
        
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0);
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), comp));        
        armDoublePH.set(Value.kReverse);
        slideDoublePH.set(Value.kReverse);
       // arm pos manual input for complience
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
        if (mode_4_5_Timer.hasElapsed(.5)){
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), coneHigh));
        slideDoublePH.set(Value.kForward);

      }
      break;
    }
      case 5: // high cube
      {
        
        slideDoublePH.set(Value.kReverse);
        armDoublePH.set(Value.kForward);
        if (mode_4_5_Timer.hasElapsed(0.1)){
          m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeHigh));
        }
          break;        
      }
    
      
      case 6: { // cone mid
          slideDoublePH.set(Value.kReverse);
          armDoublePH.set(Value.kForward);
          m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), coneMid)); 
        break;
      }

      case 9: { //cube mid
        
          armDoublePH.set(Value.kReverse);
          slideDoublePH.set(Value.kReverse); 
          m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeMid)); 
          break;
      }


//#########################################################################################################      
      case 7: { //outfeed cone
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, -.5); 
       break;

      }
      case 8: { //outfeed cube
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 2);//0.75
        break;

      }
//################################################################################################################
// adjustment modes

      case 25: { //cone adjustment in/cube out
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, -.3); 
        if (sensor_Timer.hasElapsed(.3)){
          m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0); 
        }
        break;
      }
      case 26: { //cube adjustment in/ cone out(be carefull!!)
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, .3); 
        if (Sensor.get() & sensor_Timer.hasElapsed(.3)){
          m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0); 
        }
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
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), 0.43));
        armDoublePH.set(Value.kForward);

        break;
      }

      case 15:{//cube wrist up
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), cubeHigh));
        armDoublePH.set(Value.kForward);

        break;
      }
      case 16: {
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, -0.4); 
          break;
      }

      case 18: {
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 2); 
          break;
      }

      case 17:{
      
          m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), 0.45));// wrist pos
          armDoublePH.set(Value.kReverse);// arm pos
  
          m_infeedMotor.set(TalonFXControlMode.PercentOutput, -0.5);// motor speed
          if (Sensor.get()) {// sensor delay
            sensor_Timer.start();
          } else {
            sensor_Timer.reset();}

          if (Sensor.get() && sensor_Timer.hasElapsed(.3 )) {
            mode = 19;
          }
          break; 
      }

      case 19:{
   
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0);
        m_wristMotor.set(TalonFXControlMode.PercentOutput, wristPID.getOutput(wristPot.get(), 0.8 ));        
        armDoublePH.set(Value.kReverse);
        slideDoublePH.set(Value.kReverse);
       
        break;
      }

      case 20:{
        armDoublePH.set(Value.kReverse);// arm pos manual input for complience

      }

      case 23:{ //auto slide out
        slideDoublePH.set(Value.kForward);

      }
      case 24:{ //auto slide out
        slideDoublePH.set(Value.kReverse);

      }

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
    
    wristPID.setP(3.215);
    wristPID.setI(0.03605);//0.057
    wristPID.setD(15);//20
    // wristPID.setF(0); //dont toutch unless you know what it will do
    wristPID.setOutputLimits(-.6, .6);//.45
    wristPID.setOutputRampRate(0.0777); 
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
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

     pcmCompressor.enableAnalog(0, 120);
      
      armDoublePH.set(Value.kReverse);
      slideDoublePH.set(Value.kReverse);
      setMode(0);
      wristPID.reset();


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
    // SmartDashboard.putNumber("wristMotorPID output", wristPIDOutput);
    SmartDashboard.putNumber("mode", getMode());
    // SmartDashboard.putNumber("memory_mode", memory_mode);
    SmartDashboard.putBoolean("compressor", pcmCompressor.isEnabled());
    SmartDashboard.putBoolean("Auto Enabled", autoEnabled);
    SmartDashboard.putNumber("airPressure", pcmCompressor.getPressure());
    // SmartDashboard.putNumber("WristPid", wristPIDOutput);
    SmartDashboard.putBoolean("sensor", Sensor.get());
    SmartDashboard.putNumber("Motor Speed", wristPID.getOutput());



    // SmartDashboard.putNumber("swerveAngle");



    // else {m_infeedMotor.set(0);}
    if (!autoEnabled) {
     
      if (m_drivController.getRawButtonPressed(5) || (m_flightStick.getRawButtonPressed(7))) {// panic button  (9 for xbox, 5 for joystick)
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 0);
        setMode(0);
      }
      if (m_flightStick.getRawButtonPressed(8)){
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
      // if (m_drivController.getRawButtonPressed(16) && (getMode() !=4 || getMode() != 5)){
      //   setMode(21);
      // }
      // if (m_drivController.getRawButtonPressed(15) && (getMode() !=4 || getMode() != 5)){
      //   setMode(22);
      // }

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
      if (m_drivController.getRawButton(15)){// low cone (14 for joustick  7 for xbox )
        m_infeedMotor.set(TalonFXControlMode.PercentOutput, 3);
      }
      
//##############################################################################################

    //adjustment buttons
    if (m_drivController.getRawButtonPressed(8)){
      setMode(25);
      sensor_Timer.reset();
      sensor_Timer.start();
    }
    if (m_drivController.getRawButtonPressed(9)){
      setMode(26);
      sensor_Timer.reset();
      sensor_Timer.start();
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

      if (m_flightStick.getRawButtonPressed(9)) { // enable compressor
        pcmCompressor.enableAnalog(105, 110);}
     
       else if (m_flightStick.getRawButton(8)) { 
        pcmCompressor.disable();
      }
        

      

      if (m_flightStick.getRawButtonPressed(5)) {
        armDoublePH.set(Value.kForward);
      }
      if (m_flightStick.getRawButtonPressed(6)) {
        armDoublePH.set(Value.kReverse);
      }
      if (m_flightStick.getRawButtonPressed(1)) {
        slideDoublePH.set(Value.kForward);
      }
      if (m_flightStick.getRawButtonPressed(2)) {
        slideDoublePH.set(Value.kReverse);
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


