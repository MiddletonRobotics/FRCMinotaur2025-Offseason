package frc.minolib.interfaces;

import java.util.Optional;

import org.photonvision.simulation.PhotonCameraSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.minolib.vision.PipelineConfiguration;
import frc.minolib.vision.PipelineVisionPacket;
import frc.minolib.wpilib.Fiducial;

public interface MinoVisionCamera {
    /** Returns the simulated camera object. */
    public PhotonCameraSim getCameraSim();
  
    public void updateInputs();
  
    /** Returns the latest measurement. */
    public PipelineVisionPacket getLatestMeasurement();
  
    /** Select the active pipeline index. */
    public void setPipelineIndex(int index);
  
    /** Get the active pipeline config. */
    public PipelineConfiguration getPipelineConfiguration();
  
    /** Returns the robot-to-camera transform. */
    public Transform3d getTransform();
  
    public Optional<Matrix<N3, N3>> getCameraMatrix();
  
    public Optional<Matrix<N8, N1>> getDistCoeffs();
  
    /** Returns the type of fiducials this camera is tracking. */
    public Fiducial.Type getFiducialType();
  }