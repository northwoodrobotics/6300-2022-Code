package frc.ExternalLib.NorthwoodLib.NorthwoodControllers;
import edu.wpi.first.wpilibj.Joystick;
import frc.ExternalLib.JackInTheBotLib.robot.input.Axis;
import frc.ExternalLib.JackInTheBotLib.robot.input.JoystickAxis;
import frc.ExternalLib.SpectrumLib.controllers.SpectrumThumbStick;
import frc.ExternalLib.SpectrumLib.controllers.SpectrumXboxController;
public class NWXboxController extends SpectrumXboxController{

    private final Axis leftXAxis;
	private final Axis leftYAxis;
    private final Axis rightXAxis;
	private final Axis rightYAxis;
    private final Joystick controller;




    public NWXboxController(int port){
        super(port);
        controller = new SpectrumXboxController(this.getPort());

        leftXAxis = new JoystickAxis(controller, 0);
        leftYAxis = new JoystickAxis(controller, 1);
        rightXAxis = new JoystickAxis(controller, 4);
        rightYAxis= new JoystickAxis(controller, 5);


        
    }public NWXboxController(int port, double xDeadband, double yDeadband) {
		this(port);
		this.leftStick.setDeadband(xDeadband, yDeadband);
		this.rightStick.setDeadband(xDeadband, yDeadband);
	}
    
	
	public Axis getLeftXAxis() {
		return leftXAxis;
	}


	public Axis getLeftYAxis() {
		return leftYAxis;
	}
    public Axis getRightXAxis() {
		return rightXAxis;
	}
    public Axis getRightYAxis() {
		return rightYAxis;
	}




    






    
}
