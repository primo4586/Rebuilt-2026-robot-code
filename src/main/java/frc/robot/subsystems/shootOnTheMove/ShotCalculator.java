package frc.robot.subsystems.shootOnTheMove;

import java.util.function.DoubleSupplier;

import com.techhounds.houndutil.houndlib.ChassisAccelerations;
import com.techhounds.houndutil.houndlib.ShootOnTheFlyCalculator;
import com.techhounds.houndutil.houndlib.ShootOnTheFlyCalculator.InterceptSolution;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.annotations.Tunable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CommandGroupFactory;
import frc.robot.primoLib.PrimoCalc;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.interpolation.InterpolateUtil;

// stores current target and actively computes effective target
@LoggedObject
public class ShotCalculator extends SubsystemBase {
    
    private Drive drivetrain = Drive.getInstance(RobotBase.isReal());

    //singleton
    private static ShotCalculator instance;
    public static ShotCalculator getInstance(){
        if(instance == null){
            instance = new ShotCalculator(Drive.getInstance(RobotBase.isReal()));
        }
        return instance;
    }


 


    @Log
    private Pose3d currentEffectiveTargetPose = Pose3d.kZero;

    @Log
    private double currentEffectiveYaw;

    private InterceptSolution currentInterceptSolution;

    @Log
    private Pose3d targetLocation = PrimoCalc.getHubPos();

    @Log
    private double targetDistance = 0.0;

    @Log
    @Tunable
    private double targetSpeedRps = 8;

    public ShotCalculator(Drive drivetrain) {
        this.drivetrain = drivetrain;
    }

    
    @Override
    public void periodic() {
        Pose2d drivetrainPose = drivetrain.getPose();
        targetDistance = drivetrainPose.getTranslation().getDistance(targetLocation.toPose2d().getTranslation());
        
        // 1. Get the map value
        targetSpeedRps = InterpolateUtil.interpolate(
        ShooterConstants.SHOOTER_INTERPOLATION_MAP, PrimoCalc.getDistance(drivetrainPose, PrimoCalc.getHubPos()));
    
        Pose3d shooterPose = new Pose3d(drivetrainPose).plus(new Transform3d()); 
        ChassisSpeeds drivetrainSpeeds = drivetrain.getFieldRelativeSpeeds();
        ChassisAccelerations drivetrainAccelerations = drivetrain.getFieldRelativeAccelerations();

        currentInterceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(
                shooterPose, targetLocation, drivetrainSpeeds, drivetrainAccelerations, targetSpeedRps, 5, 0.01);
    
    }

    public void setTarget(Pose3d targetLocation, double targetSpeedRps) {
        this.targetLocation = targetLocation;
        this.targetSpeedRps = targetSpeedRps;
    }

    public Pose3d getCurrentEffectiveTargetPose() {
        return currentEffectiveTargetPose;
    }

    public double getCurrentEffectiveYaw() {
        return currentEffectiveYaw;
    }

    public InterceptSolution getInterceptSolution() {
        return currentInterceptSolution;
    }
}