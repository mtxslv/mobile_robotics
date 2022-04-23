# Intro
Hello everyone! This file aims to explain what I've done so far. Probably it will be modified in the future. Whatever...

# Updating...
The first thing ever I did was to create this folder. The [recommended video](https://youtu.be/SQont-mTnfM) used anaconda associated with PythonXY as Programming Interface. Since I had jupyter in VSCode, I employed it because it has the same functionalities as PythonXY.

I then created the `test_1.ipynb` file. 

I moved the files `sim.py`, `simConst.py` and `simpleTest.py` to this folder from `CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/programming/remoteApiBindings/python/python/` and the file `remoteApi.so` to here from `/CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/programming/remoteApiBindings/lib/lib/Ubuntu20_04`. 

The file `simConst.py` is a necessary file for `sim.py` (another library file). The `simpleTest.py` is self-explainatory. It exemplifies basic ideas behind the coding process. The file `remoteApi.so` is necessary for connecting to the simulator.

I then went for the simulator. I run it, putted a mobile robot in the scene and disabled its script (click script in the left hand side menu -> click in `Child script "/PionerP3DX"` in the pop up and then checkd the disabled option).

I double-clicked the scene script and added `simRemoteApi.start(19999)` after `require('defaultMainScript')`. Ideally, the line of code added should be in a scene object, but I couldn't make it work, so I tried adding the line directly to the scene and it worked. If you make it work somehow else, teach us!

Even though I just provided details on the scene, it can be imported directly on the simulator. The file containing the scene is called `teste_movimento_1.ttt`

Now, in order to connect your code to the simulator, just start the simulation and run your thing! Ezy Pzy LmonSkweezy. But Watch Out! It **has** to be done this way. If you try to start the sim only after running your code already, it won't work. 

# References:
- [CoppeliaSim Functions' Docs](https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm)
- [A Reference On Pioneer Robot](https://cyberbotics.com/doc/guide/pioneer-3dx)
