In order to get [robotics toolbox](https://github.com/petercorke/robotics-toolbox-python) ready and running I needed to fix some libraries stuff. Here I'll explain all the steps.

1. First, install the Robotics Toolbox. I'm using Python 3.8.10, so I did: ```python3.8 -m pip install roboticstoolbox-python```
2. Update Numpy to latest version (1.23.1 worked): ```python3.8 -m pip install numpy --upgrade```
3. Sympy version must be 1.10.1 so _sympy.core.symbol_ work: ```python3.8 -m pip install numpy --upgrade```

Everything is fine.