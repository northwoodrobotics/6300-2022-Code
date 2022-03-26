package frc.swervelib.ctre;

//import com.ctre.phoenix.sensors.PigeonIMUSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.swervelib.Gyroscope;

public class Pigeon2FactoryBuilder {
    //private static PigeonIMUSimCollection pigeonSim;

    public Gyroscope build(WPI_Pigeon2 pigeon) {
        return new GyroscopeImplementation(pigeon);
    }

    private static class GyroscopeImplementation implements Gyroscope {
        private final WPI_Pigeon2 pigeon;
        

        private GyroscopeImplementation(WPI_Pigeon2 pigeon) {
            this.pigeon = pigeon;
           // pigeonSim = pigeon.getSimCollection();
        }
        @Override 
        public void calibrateGyroscope(){
            pigeon.calibrate();
        }

        @Override
        public Rotation2d getGyroHeading() {
            return Rotation2d.fromDegrees(pigeon.getYaw());
        }
        @Override
        public double readGetAngle(){
            return pigeon.getAngle();
        }
        @Override
        public Rotation2d readGetYaw(){
            return Rotation2d.fromDegrees(pigeon.getYaw());
        }
        @Override
        public Rotation2d readFused(){
            return Rotation2d.fromDegrees(pigeon.getYaw());
        }
        

        @Override
        public void zeroGyroscope() {
            pigeon.setYaw(0.0);
        }

        @Override
        public void setAngle(double angle) {
            //pigeonSim.setRawHeading(angle);
        }
    }
}
