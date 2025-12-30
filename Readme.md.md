## Vehicle Control models:



* 0\_Load\_data is the common load file for all models.



* a\_Rajdeep\_7\_DOF\_model : Rajdeep's 7-DOF model for longitudinal and lateral vehicle dynamics.



* b\_Camber\_Caster\_Toe\_added : Added camber, caster and toe effects on vehicle dynamics and also added how these angles change with compliance.



* c\_Suspension\_kinematics\_added : Removed load transfer based on ax and ay and added suspension deflection to the model and calculated load\_transfer from the same, Roll and Pitch angle were calculated from each suspension displacement.



###### In all above models, there was a problem of acceleration oscillation in the beginning.



* d\_proper\_suspension\_kinematics: Addressing the problem of acceleration oscillation, We added the transfer function block and set the value of time constant calculated from spring stiffness and vehicle\_mass.



* e\_aerodynamics\_added : Added effect of aerodynamic downforce on vertical load of each tire, pitching moment due to downforce.
  

* f\_updated\_seven\_DOF\_vehicle\_model: Added suspension displacement due to aerodynamic downforce in previous aerodynamic model added saturation to omega as a substitution to the powertrain limits(omega saturated to 47.5 m/sec).


* f\_updated\_vehicle\_model: Subsystem wise loading of parameters for better user experience in the app. Changes were made in codes of matlab function accordingly.


