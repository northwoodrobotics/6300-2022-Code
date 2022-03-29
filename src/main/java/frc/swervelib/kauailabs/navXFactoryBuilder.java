package frc.swervelib.kauailabs;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.swervelib.Gyroscope;

public class navXFactoryBuilder {
    
    public Gyroscope build(AHRS navX) {
        return new GyroscopeImplementation(navX);
    }

    private static class GyroscopeImplementation implements Gyroscope {
        private final AHRS navX;
        private final SimDouble angleSim;
        private final SimDouble FusedSim;

        private GyroscopeImplementation(AHRS navX) {
            this.navX = navX;
            

            navX.calibrate();

            int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            angleSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
            FusedSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "FusedHeading"));
        }

     /*   @Override public void calibrateGyroscope(){
            navX.calibrate();
        }*/

        @Override   
        public Rotation2d getGyroHeading() {
            if (navX.isMagnetometerCalibrated()){
                return Rotation2d.fromDegrees(-navX.getFusedHeading());
            }
     
        return Rotation2d.fromDegrees(360-navX.getYaw());
    }

        @Override
        public void zeroGyroscope() {
            navX.reset();
        }

        @Override
        public double readGetAngle(){
            return 360- navX.getAngle();
        }
        @Override
        public Rotation2d readGetYaw(){
            return Rotation2d.fromDegrees(navX.getYaw());
        }
        @Override
        public Rotation2d readFused(){
            return Rotation2d.fromDegrees(360-navX.getFusedHeading());
        }
        



        @Override
        public void setAngle(double angle) {
            angleSim.set(angle);
        }
    }
}
