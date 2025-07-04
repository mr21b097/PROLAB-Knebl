# PROLAB-Knebl


In der zip ist nochmals das gesamte Projekt, ansonsten ist example_package bereits das package.
Ich entschuldige mich für die gebliebene Platzhalter-Benennung, jedoch bin ich aufgrund privater
Umstände zu einer etwas verfrühten Abgabe gezwungen.



<--  Filter kombiniert -->


roslaunch example_package start_filters.launch 



<--  Nodes einzeln -->

1. Launch für Rviz und Gazebo
   
roslaunch example_package start.launch

3. Nodes
rosrun example_package ...
- filter_node_KF    
- filter_node_EKF   
- filter_node_PF   
(- pose_logger_node)

