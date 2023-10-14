package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Signaling;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.*;
import com.pathplanner.*;



public class AutoBalanceAuto extends SequentialCommandGroup {
    public AutoBalanceAuto(Swerve s_Swerve){

 
//#######################################################################################################################


//loads path
PathPlannerTrajectory autoBalance = PathPlanner.loadPath("autoBalance", new PathConstraints(1, 0.6));

//creates swerve controller command
PPSwerveControllerCommand swerveControllerCommand =
new PPSwerveControllerCommand(
    autoBalance,
    s_Swerve::getPose,
    Constants.Swerve.swerveKinematics,
    new PIDController(2, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.   
    //TODO try 0.5, 5, 0 at comp
    new PIDController(2, 0, 0), // Y controller (usually the same values as X controller)

    new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    s_Swerve::setModuleStates,
    false,
    s_Swerve);


//Commands
//####################################################################################################################################

addCommands(
    new InstantCommand(() -> {s_Swerve.zeroGyro();})//zeroGyro
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 13;})//arms up
);

addCommands(
    new WaitCommand(.5)
);
addCommands(
    new InstantCommand(() -> {Signaling.mode = 14;})//wrist out cone
    
);

addCommands(
    new WaitCommand(1.25)
);

addCommands(
    new InstantCommand(() -> {Signaling.mode = 7;})//score cone
   
);

addCommands(
    new WaitCommand(.5) //.25
);
    
addCommands(
    new InstantCommand(() -> {Signaling.mode = 3;})//compliance
);


addCommands(
    new InstantCommand(
        () -> s_Swerve.resetOdometry(
            new Pose2d(
                autoBalance.getInitialPose().getX(),
                autoBalance.getInitialPose().getY(),
                autoBalance.getInitialHolonomicPose().getRotation()
                ))),
                swerveControllerCommand
);

addCommands(
    new AutoBalanceCommand(s_Swerve)//autobalance
    );

addCommands(
    new InstantCommand(() -> s_Swerve.zeroGyroAuto())//set gyro 180
    );


    }
}