# Genetic Algorithm Project
### - A PID Controller for automating car driving simulation-

In this project, I made a genetic algorithm that can find the right values to be used for the PID Controller of the automatic driving car system.

The visualization for the car pathing is referenced from here:
> Sakai, A., Ingram, D., Dinius, J., Chawla, K., Raffin, A. and Paques, A., 2022. PythonRobotics: a Python code collection of robotics algorithms. [online] arXiv.org. Available at: <https://arxiv.org/abs/1808.10703> [Accessed 14 May 2022].

In this current commit the formula of the followed path would be:

![equation](https://latex.codecogs.com/png.image?%5Clarge%20%5Cdpi%7B110%7D%5Cbg%7Bwhite%7Dsin(%5Cfrac%7Bx%7D%7B5%7D)%20%5Ctimes%20x)

which can be edited on function graph() from pure_lineTracker.py

Here is a result from run testing the algorithm:

![generation 1](assets/Figure_1.png)
![generation 2](assets/Figure_2.png)
![generation 3](assets/Figure_3.png)
![generation 4](assets/Figure_4.png)
![generation 13](assets/Figure_5.png)


