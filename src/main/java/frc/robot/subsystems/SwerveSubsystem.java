package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

//import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveSubsystem extends SubsystemBase {
    //private PhotonCamera cam = new PhotonCamera(Constants.kCamName);
    double sum;
    boolean flagOverDrive;
    List<SwerveModuleState> newStates;
    double normalizedSpeed;
    private final SwerveModule frontLeft = new SwerveModule(
            Constants.kFrontLeftDriveMotorPort,
            Constants.kFrontLeftTurningMotorPort,
            Constants.kFrontLeftDriveEncoderReversed,   
            Constants.kFrontLeftTurningEncoderReversed,
            Constants.kFrontLeftDriveAbsoluteEncoderPort,
            Constants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            Constants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            Constants.kFrontRightDriveMotorPort,
            Constants.kFrontRightTurningMotorPort,
            Constants.kFrontRightDriveEncoderReversed,
            Constants.kFrontRightTurningEncoderReversed,
            Constants.kFrontRightDriveAbsoluteEncoderPort,
            Constants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            Constants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            Constants.kBackLeftDriveMotorPort,
            Constants.kBackLeftTurningMotorPort,
            Constants.kBackLeftDriveEncoderReversed,
            Constants.kBackLeftTurningEncoderReversed,
            Constants.kBackLeftDriveAbsoluteEncoderPort,
            Constants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            Constants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            Constants.kBackRightDriveMotorPort,
            Constants.kBackRightTurningMotorPort,
            Constants.kBackRightDriveEncoderReversed,
            Constants.kBackRightTurningEncoderReversed,
            Constants.kBackRightDriveAbsoluteEncoderPort,
            Constants.kBackRightDriveAbsoluteEncoderOffsetRad,
            Constants.kBackRightDriveAbsoluteEncoderReversed);

     private final AHRS gyro = new AHRS(SPI.Port.kMXP);
     private SwerveModulePosition[] swerveModPose= new SwerveModulePosition[]{
        frontLeft.getSwerveModulePosition(),
        backLeft.getSwerveModulePosition(),
        frontRight.getSwerveModulePosition(),
        backRight.getSwerveModulePosition()
     };

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0), swerveModPose);
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(Constants.kDriveKinematics, getRotation2d(), swerveModPose, getPose());

    double xinput;
    
    public SwerveSubsystem() {
        //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5,0.5,20));
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

    }
    public void updateSwerveModPose(){
        this.swerveModPose = new SwerveModulePosition[]{
            frontLeft.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backRight.getSwerveModulePosition()
     };
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
       return Math.IEEEremainder(gyro.getAngle(), 360);
       //return 0;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),swerveModPose ,pose);
    }

    @Override
   
   public void periodic() {   
    SmartDashboard.putNumber("Robot Heading", getHeading());
}
    /*private void updateShuffleBoard(){
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putString("estimated pose", poseEstimator.getEstimatedPosition().getTranslation().toString());
        SmartDashboard.putNumber("angle", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("fl encoder angle", frontLeft.getAbsoluteEncoderRad());
    }*/

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    public double getX(){
        return xinput;
    }
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        //what it was supposed to do
        //Constants.kDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
        
    }
}
