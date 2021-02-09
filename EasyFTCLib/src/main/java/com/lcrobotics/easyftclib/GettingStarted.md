# Using EasyFTC

Welcome to EasyFTC!
This library's goal is to make using FTC's existing codebase more adaptable and user-friendly. It's
overall structure is very similar to FRC's WPILib, also using systems and subsystems. To find out more
about command based programming check out the [FRC docs](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)

## How to Use the Existing CodeBase
1. We've rewritten the DcMotor class to be more user friendly, so when declsaaring motors, you'll 
want to use ```Motor [name]``` instead of ```DcMotor [name]```.

2. We've written an example subsystem for mecanum driving. To use it, you need to declare the motors and
the MecanumDrive constructor outside of init()and loop(). In init() you should initialize the motors
and MecanumDrive constructor. In loop() you can call the methods you need from the MecanumDrive class. 
For example code, see EasyFTCLib/java/exampleCode

3. You can add your own systems and subsystems to your codebase! The goal of EasyFTC is to make FTC's
library more accessible.