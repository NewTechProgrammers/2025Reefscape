// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;

// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ModuleConstants;

// public class CamModule extends SubsystemBase {
//     public PhotonCamera camera;

//     public CamModule(String camName)
//     {
//         try {
//             camera = new PhotonCamera("photonvision");
//         }
//         catch(Exception e){
//             System.out.println("Problem with camera occured");
//             return;
//         }
        
        

//     }

//     public void dataTake(){
//         var result = camera.getLatestResult();
//         if(result.hasTargets()){

//         }
//     }

// }