# Trajectory-Control

Here I provide a simple but highly accurate trajectory control method for self-driving cars. The simulation is based on the Carla's /Game/Maps/RaceTrack, where you can follow the Coursera's Self-driving Specialization course-1 final [project](https://www.coursera.org/learn/intro-self-driving-cars/programming/ac8R5/final-project-self-driving-vehicle-control) to setup.

Here is the overview of the result without careful tuning. The controlled trajectory perfectly overlaps on the desired/reference trajectory.
![trajectory](https://user-images.githubusercontent.com/29236300/132007771-343b777b-63c6-4c09-a578-3f536f083e05.png)
Zoom in 400% for the details in the turns/arcs:
![image](https://user-images.githubusercontent.com/29236300/132008157-d7218a0d-bf13-4cba-b13d-cc330fdcf79d.png)
![image](https://user-images.githubusercontent.com/29236300/132008277-e987be31-189c-458a-a872-ec3ffa04921c.png)
![image](https://user-images.githubusercontent.com/29236300/132008330-e8cc4138-9756-4725-ae0f-9b125bccd3fb.png)

As for the longitudinal control, it's just a simply PID controller.
