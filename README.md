# DroneFlyer

DroneFlyer is a 3D drone simulation program designed to help pilots of a hypothetical drone.
The user presses four keys to control the four engines of the quadcopter drone, and tries to reach targets
placed around the environment. Our program uses a custom perspective-based 3D rendering system supported by
the libraries pygame and numpy. The program also shows a plot of acceleration, velocity, and position over
time, using matplotlib.

## Execution

Install the dependencies from pyproject.toml, and then run the python file `droneflyer.py`. If you are using
`uv`, you can run the command `uv run droneflyer.py`
