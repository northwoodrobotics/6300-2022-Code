package frc.ExternalLib.NorthwoodLib.MathWrappers;

import edu.wpi.first.math.geometry.Translation2d;
import frc.ExternalLib.JackInTheBotLib.math.MathUtils;
import frc.ExternalLib.JackInTheBotLib.util.Interpolable;
import edu.wpi.first.math.geometry.Rotation2d;

import java.io.Serializable;
import java.text.DecimalFormat;
import java.util.Objects;


public final class NWTranslation2d extends Translation2d implements Serializable, Interpolable<NWTranslation2d>{
    public static final long serialVersionUID = 7566662924062254723L;


    public static final NWTranslation2d ZERO = new NWTranslation2d(0.0, 0.0);


    public final double m_x;
    public final double m_y;
    public final double length;

    

    public NWTranslation2d(double x,double y){
        this.m_x = x;
        this.m_y = y;
        this.length = Math.hypot(x, y);

    }

    public double getLength(){
        return length;
    }


    public NWTranslation2d add(double x, double y){
        return new NWTranslation2d(this.m_x+x, this.m_y+y);
    }

    public NWRotation2d getAngle(){
        return new NWRotation2d(m_x,m_y);
    }

    public static NWTranslation2d fromAngle(NWRotation2d rotation){
        return new NWTranslation2d(rotation.getCos(), rotation.getSin());
    }

    public NWTranslation2d normal(){
        return new NWTranslation2d(m_x/length, m_y/length);
    }

    public double cross(NWTranslation2d other){
        return m_x*other.m_y-m_y*other.m_x;
    }
	public double dot(NWTranslation2d other) {
		return m_x * other.m_x + m_y * other.m_y;
	}












    





    //@Override 
    @Override
    public NWTranslation2d NWinterpolate(NWTranslation2d other, double t){
        if (t <= 0.0) {
			return this;
		} else if (t >= 1.0) {
			return other;
		} else {
			return extrapolate(other, t);
		}
    }
    
  

    
    public NWTranslation2d extrapolate(NWTranslation2d other, double t){
        NWTranslation2d delta = (NWTranslation2d) other.minus(this);
        return (NWTranslation2d) this.plus((delta.times(t)));
    }



    @Override
    public boolean equals(Object obj){
        if(!(obj instanceof NWTranslation2d)){
            return false;
        }
        return equals((NWTranslation2d) obj, MathUtils.EPSILON);
    }


    public boolean equals(NWTranslation2d other, double allowableError){
        return MathUtils.epsilonEquals(getX(), other.getY(), allowableError) &&
                MathUtils.epsilonEquals(getY(), other.getY(), allowableError);

    }

    public String toString(){
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return '('+ fmt.format(getX()) + "," + fmt.format(getY())+ ')';
    }


    public static NWRotation2d getAngleBetween(NWTranslation2d a, NWTranslation2d b) {
		double cos = a.dot(b) / (a.length * b.length);
		if (Double.isNaN(cos)) {
			return NWRotation2d.ZERO;
		}

		return NWRotation2d.fromRadians(Math.acos(MathUtils.clamp(cos, -1.0, 1.0)));
	}






    









    
}
