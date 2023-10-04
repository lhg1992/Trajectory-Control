# Trajectory Control

Here I provide a simple but highly accurate trajectory(lateral) control method for self-driving cars. The simulation is based on the CarlaSimulator /Game/Maps/RaceTrack, where you can follow the Coursera's Self-driving Specialization course-1 [final project](https://www.coursera.org/learn/intro-self-driving-cars/programming/ac8R5/final-project-self-driving-vehicle-control) to setup.

The overview of the result without careful tuning is shown below. The controlled trajectory(orange line) perfectly overlaps on the desired/reference trajectory(green line).
![trajectory](https://user-images.githubusercontent.com/29236300/132007771-343b777b-63c6-4c09-a578-3f536f083e05.png)
Zoom in 400% for the details in the turns/arcs:
![image](https://user-images.githubusercontent.com/29236300/132008157-d7218a0d-bf13-4cba-b13d-cc330fdcf79d.png)
![image](https://user-images.githubusercontent.com/29236300/132008277-e987be31-189c-458a-a872-ec3ffa04921c.png)
![image](https://user-images.githubusercontent.com/29236300/132008330-e8cc4138-9756-4725-ae0f-9b125bccd3fb.png)

**The core idea is the dynamic lookahead length based on a lookahead time(around 0.5s) and current velocity for the "next waypoint", rather than a fixed lookahead length.** Then use the Stanley method as the course suggests. The pursuit method should also work.

As for the longitudinal control, it's just a simple PID controller as the course suggests, additionally I defined a simple smoother for throttle control for comfort consideration. Therefore it doesn't follow the desired/reference velocity so well at the velocity spikes, as shown below, blue line is the desired velocity:
![image](https://user-images.githubusercontent.com/29236300/132018623-ecd4181a-9357-4cf3-8c91-ff161873b6f2.png)
