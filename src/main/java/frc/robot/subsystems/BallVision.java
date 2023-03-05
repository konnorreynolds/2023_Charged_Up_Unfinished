// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.VisionConstants;

public class BallVision extends SubsystemBase {

    // Initializes Photonvision Camera
    private PhotonCamera camera = new PhotonCamera(VisionConstants.camera);
    // Creates an instance of BallVision to be used below
    public static BallVision instance;
    // Initializes camera and target positions
    private double yawVal=0;
    private double pitchVal=0;
    private double skewVal=0;
    private double areaVal=0;
    private boolean hasTarget = false;
    
    
    public BallVision() {
    }

    public static BallVision getInstance() {
        if (instance == null) {
          instance = new BallVision();
        }
        return instance;
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        // Gives the target position values
        var result = this.camera.getLatestResult();
        if (result.hasTargets()) {
            this.yawVal = result.getBestTarget().getYaw();
            this.pitchVal = result.getBestTarget().getPitch();
            this.skewVal = result.getBestTarget().getSkew();
            this.areaVal = result.getBestTarget().getArea();
            this.hasTarget =true;

            SmartDashboard.putNumber("Ball Yaw Value", yawVal);
            
        }
        else{
            this.hasTarget = false;
        }
 
    }

    // Returns the camera for other subsystems or commands
    public PhotonCamera camera(){
        return camera;
    }

    // Sets conePipeline to the camera for quickswap vision targets
    public void conePipe(){
        camera.setPipelineIndex(0);
    }

    // Sets cubePipeline to the camera for quickswap vision targets
    public void cubePipe(){
        camera.setPipelineIndex(1);
    }

    // Returns target values for other subsytems or commands
    public double getYawVal(){
        return this.yawVal;
    }

    public double getPitchVal(){
        return this.pitchVal;
    }

    public double getSkewVal(){
        return this.skewVal;
    }

    public double getAreaVal(){
        return this.areaVal;
    }

    public boolean hasTargets(){
        return this.hasTarget;
    }

    // Returns if photonvision has targets or not
    public List<PhotonTrackedTarget> getTargets() {
        List<PhotonTrackedTarget> targets = null;
        var results = this.camera.getLatestResult();
        if(results.hasTargets()) {
            targets = results.getTargets();
        }
        return targets;
    }
}
