// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.video.Video;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSink.Kind;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  private UsbCamera m_driverCamera;
  private UsbCamera m_floorCamera;
  private MjpegServer m_mjpegServer;
  private boolean m_bsource; // true is driver cam
  

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {
    m_driverCamera = new UsbCamera("Driver", 0);
    m_floorCamera = new UsbCamera("Floor", 1);
    m_bsource = true;
    m_mjpegServer = new MjpegServer("Camera Server", 1181);
    CameraServer.startAutomaticCapture();
  }

  public void toggleCamera() {
    if(m_bsource){
      m_mjpegServer.setSource(m_floorCamera);
      m_bsource = false;
    }
    else{
      m_mjpegServer.setSource(m_driverCamera);
      m_bsource = true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
