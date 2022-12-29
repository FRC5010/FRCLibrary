# 5010 FRCLibrary
The 5010 FRC Library is a collection of classes and examples projects, which can be used as the starting basis for a robot project, and contains the collections of code that 5010 Tiger Dynasty has found useful to have on hand from one build season to another.

# Generic Classes
A number of classes in this library are prefixed with the title, 'Generic'. These classes are meant to be the primary way to utilize functionality that is being supported. They will either be abstract classes or interfaces, so they will have a lot of unimplemented functions and just a few member variables. Although, the main robot code will use variables of the Generic types, the actual values of the variables will be specific derivations of each Generic types that correspond to the actual functionality that is being used.<p>

For example: <code>GenericGyro gyro = new NavXGyro(0);</code><p>

Code using the <code>gyro</code> variable would use functions from the <code>GenericGyro</code> class to access <b>Gyro</b> functionality and the <code>NavXGyro</code> class would implement those functions in a way that is compatible with how a <b>NavX gyro</b> works.<p>

This method of coding is called a <b>Wrapper pattern</b> because the real and specific functionality is hidden behind the more general wrapper class. One key advantage of doing this is that another specific functionality class can be written to support different underlying hardware, such as a <b>Pigeon</b> gyro or the <b>Analog</b> gyro, or even to provide a <i>Simulated</i> gyro when not running code on the robot.<p>
The specific implementation is selected when the robot code is being initialized in <code>RobotContainer</code> or other classes which may be part of the initialization code.  

# Mechanism Classes
FRC5010 divides our code into <i>mechanisms</i> for the purpose of separating our coding efforts more effectively and ensuring that multiple developers can work on code without interfering with each other. There is a <code>GenericMechanism</code> class which we use to enforce the use of a particular set of functions across each mechanism subdivision. If information from one mechanism is needed by code in another, the mechanism should provide a way to access it by providing it through a shared class.
  
# Persisted Class
The <code>Persisted</code> class is a wrapper around the WPILib Preferences functionality that hides some of the boilerplate code needed to use Preferences. Perferences can be accessed from the Dashboards under the NetworkTables/Preferences and can be updated live. This is the main use-case, so values that we want to be able to tweak live are the best ones to use it for. Tuneable things like PID values, for example. Persisted is also a type-generic class (not the same as the Generic wrapper class, but similar idea) in that it is specified using <code>Persisted&lt;TYPE&gt;</code>, see the examples below. TYPE can be Double, Integer, Boolean, String, Long or Float<p>
To use Persisted, you will need to do 3 things:
<ul>
  <li> Specify a String in a ConstantsDef.java file, notice the use of upper/lower case and underscores: <p>
<code>public class ConstantsDef {
        public static String NAME_OF_A_CONSTANT_VALUE = "NameOfAConstantValue";
    }</code>
  </li>
  <li> Instantiate the value during the initialization phase of the robot code. This can be done in a Constants file that is instantiated.<p>
  <code>public class Constants {
        public Persisted&lt;Double&gt; nameOfAConstantValue;
        public Constants() {
          nameOfAConstantValue = new Persisted<>(ConstantsDef.NAME_OF_A_CONSTANT_VALUE, 12.345);
        }
      }
</code>   
  </li>
  <li> Finally, to use it in your code, use one of the methods shown below as examples<p>
<code>public class RobotSubsystem {
    // To use the value with direct get/set functions, declare it as a Persisted<TYPE> again and instantiate as below
    private Persisted&lt;Double&gt; nameOfAConstantValue;
    public RobotSubsystem() {
      nameOfAConstantValue = new Persisted&lt;Double&gt;(ConstantsDef.NAME_OF_A_CONSTANT_VALUE, Double.class);
      nameOfAConstantValue.get(); // To use it, call the getter. Do not store this in another class variable.
      nameOfAConstantValue.set(123.45); // To set it
      // To use the value directly from a static function
      Double nameOfAConstantValue2 = Persisted.getDouble(ConstantsDef.NAME_OF_A_CONSTANT_VALUE);
    }
  }
</code>
  </li>  
</ul>  

# Vision Classes

# Motor Classes

# Controller Classes
The <code>Controller</code> class provides standard functionality for setting up joystick axis and buttons. Especially with axis, we like to have different behavior characteristics that we can add or remove easily such as cubing the input, deadzone handling, limiting the max, slewrate, etc... This class has a nested <code>Axis</code> class which is a wrapper class for handling these specific algorithms in a standard way, sometimes referred to as Decorator and Builder patterns. An example might look like this:<p>
<code>public class RobotContainer() {
    private Controller driver = new Controller(0);
    public RobotContainer() {
      driver.setLeftYAxis(driver.createLeftYAxis().deadzone(0.075).limit(0.5).rate(0.25));
      // Later on in the code, when you need the value this will use all of the algoritms defined above.
      driver.getLeftYAxis();
    }
}</code>
  
# Visualization

# Simulation
