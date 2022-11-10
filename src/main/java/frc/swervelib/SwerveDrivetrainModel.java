package frc.swervelib;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;

public class SwerveDrivetrainModel {

    QuadSwerveSim swerveDt;
    ArrayList<SwerveModule> realModules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);
    ArrayList<SwerveModuleSim> modules = new ArrayList<SwerveModuleSim>(QuadSwerveSim.NUM_MODULES);

    ArrayList<SteerController> steerMotorControllers = new ArrayList<SteerController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<DriveController> driveMotorControllers = new ArrayList<DriveController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<AbsoluteEncoder> steerEncoders = new ArrayList<AbsoluteEncoder>(QuadSwerveSim.NUM_MODULES);

    Gyroscope gyro;

    Timer keepAngleTimer = new Timer();
    double keepAngle = 0.0;       //Double to store the current target keepAngle in radians
    double timeSinceRot = 0.0;    //Double to store the time since last rotation command
    double lastRotTime = 0.0;     //Double to store the time of the last rotation command
    double timeSinceDrive = 0.0;  //Double to store the time since last translation command
    double lastDriveTime = 0.0;   //Double to store the time of the last translation command
  

    Pose2d endPose;
    PoseTelemetry dtPoseView;

    SwerveDrivePoseEstimator m_poseEstimator;
    Pose2d curEstPose = new Pose2d(SwerveConstants.DFLT_START_POSE.getTranslation(), SwerveConstants.DFLT_START_POSE.getRotation());
    Pose2d fieldPose = new Pose2d(); // Field-referenced orign
    boolean pointedDownfield = false;
    double curSpeed = 0;
    SwerveModuleState[] states;
    PIDController thetaController =
        new PIDController(
            SwerveConstants.THETACONTROLLERkP, 0, 0);

    HolonomicDriveController m_holo;




    
    private static final SendableChooser<String> orientationChooser = new SendableChooser<>();

    private double forwardSlow = 1.0;
    private double strafeSlow = 1.0;
    private double rotateSlow = 1.0;
    

   

    public SwerveDrivetrainModel(ArrayList<SwerveModule> realModules, Gyroscope gyro){
        this.gyro = gyro;
        this.realModules = realModules;

        

        if (RobotBase.isSimulation()) {
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(0)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(1)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(2)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(3)));
        }
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        endPose = SwerveConstants.DFLT_START_POSE;

        swerveDt = new QuadSwerveSim(SwerveConstants.TRACKWIDTH_METERS,
                                    SwerveConstants.TRACKLENGTH_METERS,
                                    SwerveConstants.MASS_kg,
                                    SwerveConstants.MOI_KGM2,
                                    modules);

        // Trustworthiness of the internal model of how motors should be moving
        // Measured in expected standard deviation (meters of position and degrees of
        // rotation)
        var stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(0.5));

        // Trustworthiness of gyro in radians of standard deviation.
        var localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.1));

        // Trustworthiness of the vision system
        // Measured in expected standard deviation (meters of position and degrees of
        // rotation)
        var visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

        m_poseEstimator = new SwerveDrivePoseEstimator(getGyroscopeRotation(), SwerveConstants.DFLT_START_POSE,
                SwerveConstants.KINEMATICS, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs,
                SimConstants.CTRLS_SAMPLE_RATE_SEC);

        setKnownPose(SwerveConstants.DFLT_START_POSE);

        dtPoseView = new PoseTelemetry(swerveDt, m_poseEstimator);

        // Control Orientation Chooser
        orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
        orientationChooser.addOption("Robot Oriented", "Robot Oriented");
        SmartDashboard.putData("Orientation Chooser", orientationChooser);

       
    }

    /**
     * Handles discontinuous jumps in robot pose. Used at the start of
     * autonomous, if the user manually drags the robot across the field in the
     * Field2d widget, or something similar to that.
     * @param pose The new pose the robot is "jumping" to.
     */
    public void modelReset(Pose2d pose){
        swerveDt.modelReset(pose);
    }

    /**
     * Advance the simulation forward by one step
     * @param isDisabled Boolean that indicates if the robot is in the disabled mode
     * @param batteryVoltage Amount of voltage available to the drivetrain at the current step.
     */
    public void update(boolean isDisabled, double batteryVoltage){
        // Calculate and update input voltages to each motor.
        if(isDisabled){
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                modules.get(idx).setInputVoltages(0.0, 0.0);
            }
        } else {
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                double steerVolts = realModules.get(idx).getSteerController().getOutputVoltage();
                double wheelVolts = realModules.get(idx).getDriveController().getOutputVoltage();
                modules.get(idx).setInputVoltages(wheelVolts, steerVolts);
            }
        }

        //Update the main drivetrain plant model
        swerveDt.update(SimConstants.SIM_SAMPLE_RATE_SEC);

        // Update each encoder
        for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
            double azmthShaftPos = modules.get(idx).getAzimuthEncoderPositionRev();
            double steerMotorPos = modules.get(idx).getAzimuthMotorPositionRev();
            double wheelPos = modules.get(idx).getWheelEncoderPositionRev();

            double azmthShaftVel = modules.get(idx).getAzimuthEncoderVelocityRPM();
            double steerVelocity = modules.get(idx).getAzimuthMotorVelocityRPM();
            double wheelVelocity = modules.get(idx).getWheelEncoderVelocityRPM();

            realModules.get(idx).getAbsoluteEncoder().setAbsoluteEncoder(azmthShaftPos, azmthShaftVel);
            realModules.get(idx).getSteerController().setSteerEncoder(steerMotorPos, steerVelocity);
            realModules.get(idx).getDriveController().setDriveEncoder(wheelPos, wheelVelocity);
        }

        // Update associated devices based on drivetrain motion
        gyro.setAngle(-swerveDt.getCurPose().getRotation().getDegrees());

        // Based on gyro and measured module speeds and positions, estimate where our
        // robot should have moved to.
        Pose2d prevEstPose = curEstPose;
        if (states != null) {
            curEstPose = m_poseEstimator.getEstimatedPosition();
        
            // Calculate a "speedometer" velocity in ft/sec
            Transform2d chngPose = new Transform2d(prevEstPose, curEstPose);
            curSpeed = Units.metersToFeet(chngPose.getTranslation().getNorm()) / SimConstants.CTRLS_SAMPLE_RATE_SEC;
        }
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        states = desiredStates;
        
    }
    /** set modules to X pattern, which prevents  us from being pushed. */

    public void VisionPose(Pose2d VisionMeasurement){
        m_poseEstimator.addVisionMeasurement(VisionMeasurement, Timer.getFPGATimestamp());
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param chassisSpeeds The desired SwerveModule states.
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        states = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    }
    public void driveClean(double xTranslation, double yTranslation, double rotation){
        double rot =KeepAngle(xTranslation, yTranslation, rotation);
        setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xTranslation, yTranslation, rot, getGyroscopeRotation()));
    }
    public void driveSnap(double xTranslation, double yTranslation, Rotation2d TargetAngle){
        double rot =SnapAngle(xTranslation, yTranslation, TargetAngle);
        setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xTranslation, yTranslation, rot, getGyroscopeRotation()));
    }
    
    public double KeepAngle(double xTranslation, double yTranslation, double rotation){
        double output = rotation; //Output shouold be set to the input rot command unless the Keep Angle PID is called
        if(Math.abs(rotation) >= SwerveConstants.kMinRotationCommand){  //If the driver commands the robot to rotate set the last rotate time to the current time
          lastRotTime = keepAngleTimer.get();
        }
        if( Math.abs(xTranslation) >= SwerveConstants.kMinTranslationCommand  
              || Math.abs(yTranslation) >= SwerveConstants.kMinTranslationCommand){ //if driver commands robot to translate set the last drive time to the current time
          lastDriveTime = keepAngleTimer.get();
        }
        timeSinceRot = keepAngleTimer.get()-lastRotTime;      //update variable to the current time - the last rotate time
        timeSinceDrive = keepAngleTimer.get()-lastDriveTime;  //update variable to the current time - the last drive time
        if(timeSinceRot < 0.5){                               //Update keepAngle up until 0.5s after rotate command stops to allow rotation move to finish
          keepAngle = getGyroscopeRotation().getRadians();
        }
        else if(Math.abs(rotation) < SwerveConstants.kMinRotationCommand && timeSinceDrive < 0.25){ //Run Keep angle pid until 0.75s after drive command stops to combat decel drift
          output = thetaController.calculate(getGyroscopeRotation().getRadians(), keepAngle);               //Set output command to the result of the Keep Angle PID 
        }
        return output;
    }

    public double SnapAngle(double xTranslation, double yTranslation, Rotation2d SnapAngle){
        double output = 0.0; 

           if(Math.abs(getGyroscopeRotation().getRadians()- SnapAngle.getRadians()) > SwerveConstants.kMinRotationCommand){ //Run Snap pid until 0.75s after drive command stops to combat decel drift
            output = thetaController.calculate(getGyroscopeRotation().getRadians(), SnapAngle.getRadians());               //Set output command to the result of the Keep Angle PID 
          }
          return output; 
    }

    public void setModuleStates(SwerveInput input) {
        input = handleStationary(input);

        switch (orientationChooser.getSelected()) {
            case "Field Oriented":
                states = SwerveConstants.KINEMATICS.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                input.m_translationX * SwerveConstants.MAX_FWD_REV_SPEED_MPS * forwardSlow,
                                input.m_translationY * SwerveConstants.MAX_STRAFE_SPEED_MPS * strafeSlow,
                                input.m_rotation * SwerveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC * rotateSlow,
                                getGyroscopeRotation()
                        )    
                );
                break;
            case "Robot Oriented":
                states = SwerveConstants.KINEMATICS.toSwerveModuleStates(
                        new ChassisSpeeds(
                                input.m_translationX * SwerveConstants.MAX_FWD_REV_SPEED_MPS * forwardSlow,
                                input.m_translationY * SwerveConstants.MAX_STRAFE_SPEED_MPS * strafeSlow,
                                input.m_rotation * SwerveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC * rotateSlow
                        )  
                );
                break;
        }
    }

    public SwerveModuleState[] getSwerveModuleStates() {
      return states;
    }

    public Pose2d getPose(){
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setKnownPose(Pose2d in) {
        resetWheelEncoders();
        gyro.zeroGyroscope(in.getRotation().getDegrees());
        m_poseEstimator.resetPosition(in, getGyroscopeRotation());
        curEstPose = in;
    }

    public void setKnownState(PathPlannerState initialState) {
        Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
        setKnownPose(startingPose);
        
    }
    public ChassisSpeeds getChassisSpeed(){
        return  SwerveConstants.KINEMATICS.toChassisSpeeds(states[0], states[1],states[2], states[3]);
    }

    public ChassisSpeeds getFieldRelativeSpeeds(){    
        return getChassisSpeed();
           
      }
    public ChassisSpeeds getFieldReltaiveAcceleration(){
        double lastTime = Timer.getFPGATimestamp(); 
        ChassisSpeeds PreviousSpeeds = getFieldRelativeSpeeds(); 
        return new ChassisSpeeds(
            (getFieldRelativeSpeeds().vxMetersPerSecond-PreviousSpeeds.vxMetersPerSecond)/Timer.getFPGATimestamp()-lastTime, 
            (getFieldRelativeSpeeds().vyMetersPerSecond-PreviousSpeeds.vyMetersPerSecond)/Timer.getFPGATimestamp()-lastTime,
            (getFieldRelativeSpeeds().omegaRadiansPerSecond -PreviousSpeeds.omegaRadiansPerSecond)/Timer.getFPGATimestamp()-lastTime
        );
    }
    public ChassisSpeeds getFieldRelativeJerk(){
        double lastTime = Timer.getFPGATimestamp(); 
        ChassisSpeeds PreviousSpeeds = getFieldReltaiveAcceleration(); 
        return new ChassisSpeeds(
            (getFieldReltaiveAcceleration().vxMetersPerSecond-PreviousSpeeds.vxMetersPerSecond)/Timer.getFPGATimestamp()-lastTime, 
            (getFieldReltaiveAcceleration().vyMetersPerSecond-PreviousSpeeds.vyMetersPerSecond)/Timer.getFPGATimestamp()-lastTime,
            (getFieldReltaiveAcceleration().omegaRadiansPerSecond -PreviousSpeeds.omegaRadiansPerSecond)/Timer.getFPGATimestamp()-lastTime
        );
    }
      
    

    public void zeroGyroscope() {
        gyro.zeroGyroscope(0.0);
    }
    public void plusNinetyGyroscope() {
        gyro.zeroGyroscope(90.0);
    }

    public Rotation2d getGyroscopeRotation() {
        SmartDashboard.putNumber("Gyro Angle", gyro.getGyroHeading().getDegrees());
        return gyro.getGyroHeading();
    }

    public Double GyroRoll(){
        return gyro.getGyroRoll();
    }

    public Boolean getGyroReady() {
        return gyro.getGyroReady();
    }

    public void updateTelemetry(){
        dtPoseView.update(Timer.getFPGATimestamp()*1000);
    }

    public Field2d getField() {
        return dtPoseView.getField();
    }

    public void resetWheelEncoders() {
      for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
        realModules.get(idx).resetWheelEncoder();
      }
    }

    public Command createCommandForTrajectory(PathPlannerTrajectory trajectory, SwerveSubsystem m_drive) {
        PPSwerveControllerCommand swerveControllerCommand =
            new PPSwerveControllerCommand(
                trajectory,
                () -> getPose(), // Functional interface to feed supplier
                SwerveConstants.KINEMATICS,

                // Position controllers
                SwerveConstants.XPIDCONTROLLER,
                SwerveConstants.YPIDCONTROLLER,
                thetaController,
                // feed states into controller
                commandStates -> this.states = commandStates,
                m_drive);
        return swerveControllerCommand.andThen(() -> setModuleStates(new SwerveInput(0,0,0)));
    }

    public ArrayList<SwerveModule> getRealModules() {
        return realModules;
    }

    public ArrayList<SwerveModuleSim> getModules() {
        return modules;
    }

    public void setMaxSpeeds(double forwardSlow, double strafeSlow, double rotateSlow) {
        this.forwardSlow = forwardSlow;
        this.strafeSlow = strafeSlow;
        this.rotateSlow = rotateSlow;
    }

    private SwerveInput handleStationary(SwerveInput input) {
        if (input.m_rotation == 0 && input.m_translationX == 0 && input.m_translationY == 0) {
            // Hopefully this will turn all of the modules to the "turning" configuration so being pushed is more difficult
            input.m_rotation = 0.0; //001;
        }
        return input;
    }
}