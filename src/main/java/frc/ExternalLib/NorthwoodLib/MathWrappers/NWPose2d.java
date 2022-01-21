package frc.ExternalLib.NorthwoodLib.MathWrappers;

import edu.wpi.first.math.geometry.Pose2d;
import frc.ExternalLib.JackInTheBotLib.util.Interpolable;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;

import java.io.Serializable;
import java.util.Objects;



public class NWPose2d extends Pose2d implements Serializable, Interpolable<NWPose2d> {
    public static final NWPose2d ZERO = new NWPose2d(NWTranslation2d.ZERO,NWRotation2d.ZERO );
   

    public final NWTranslation2d translation; 
    public final NWRotation2d rotation; 
    public NWPose2d(NWTranslation2d translation,NWRotation2d rotation ){
        this.translation = translation;
        this.rotation = rotation;

    }
    




    @Override
    public NWPose2d NWinterpolate(NWPose2d other, double t){
        return new NWPose2d(translation.NWinterpolate(other.translation, t), 
         rotation.NWinterpolate(other.rotation, t));
    }












    
}
