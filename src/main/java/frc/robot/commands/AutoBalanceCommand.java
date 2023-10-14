
package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoBalanceCommand extends CommandBase{

    private Swerve s_Swerve;
    private PIDController pidController;
    private Debouncer m_debouncer;

    private final double kP = 1;
    private final double kI = 0.5;
    private final double kD = 0;

    private double maxOutput = 0.75;

    private double currentAngle;
    private double output = 0.75;
    private double setPoint = 0.1;
   
    

    public AutoBalanceCommand(Swerve s_Swerve){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(1.5);

    }
    @Override
     public void initialize() {
        // s_Swerve.setSetpoint();
        m_debouncer = new Debouncer(.2, DebounceType.kRising);//0.09
        m_debouncer = new Debouncer(.2, DebounceType.kFalling);
        // pidController.reset();

    }

    @Override
    public void execute() {
        currentAngle = Math.abs(s_Swerve.getPitch())*57.2957795131; //-27.91015625
        if(pidController.getPositionError() > 5){
            maxOutput = 0.78;
        }
        if(pidController.getPositionError() < 4){
            maxOutput = 0.62;
        }
        

        output = MathUtil.clamp(pidController.calculate(currentAngle, setPoint), -maxOutput + 0.1, maxOutput - 0.05);
        s_Swerve.drive(new Translation2d(output, 0), 0, true, true);
        System.out.println(output);
        System.out.println(currentAngle + " Current");
        // System.out.println(pidController.getPositionError());
        SmartDashboard.putNumber("RobotPitch", currentAngle);

        }



    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 2, true, true);
        s_Swerve.zeroGyroAuto();
    }

    @Override
    public boolean isFinished() {
        return m_debouncer.calculate(pidController.atSetpoint());
        
    }
}