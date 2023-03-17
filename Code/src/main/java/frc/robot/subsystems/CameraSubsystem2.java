// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import org.opencv.video.Video;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.MjpegServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoSink;
// import edu.wpi.first.cscore.VideoSink.Kind;
// import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class CameraSubsystem2 extends SubsystemBase {
//   private UsbCamera m_driverCamera;
//   private UsbCamera m_floorCamera;
//   private VideoSink m_server;
//   private boolean m_bsource; // true is driver cam
  

//   /** Creates a new CameraSubsystem. */
//   public CameraSubsystem2() {
    
//     m_driverCamera =  CameraServer.startAutomaticCapture(0);
//     m_floorCamera = CameraServer.startAutomaticCapture(1);
    
//     m_server = CameraServer.getServer();

//     // m_driverCamera.setResolution(640, 480);
//     // m_floorCamera.setResolution(640, 480);
//     // m_driverCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
//     // m_floorCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);    

//     m_bsource = true;

//     CameraServer.startAutomaticCapture();
//   }

//   public void toggleCamera() {
//     if(m_bsource){
//       m_server.setSource(m_floorCamera);
//       m_bsource = false;
//     }
//     else{
//       m_server.setSource(m_driverCamera);
//       m_bsource = true;
//     }
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
