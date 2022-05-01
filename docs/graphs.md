Hi there!

This afternoon I tried to make the graphs work. Unfortunatelly, it didn't. I'll explain what I found out.

In previous versions, graph importing was easier because the we [could add the data using the UI](https://youtu.be/V1X3UJc716s). Unfortunatelly, this version requires us [to generate the graphs programatically](https://www.coppeliarobotics.com/helpFiles/en/graphs.htm).

Thus, I tried to replicate the example script (the original is seen below, followed by the one I wrote). It didn't work also, but here it goes what I learned about it.
 
```
function sysCall_init() 
    graph=sim.getObject('/Graph')
    joint1Vel=sim.addGraphStream(graph,'joint 1 velocity','deg/s',0,{1,0,0})
    joint2Vel=sim.addGraphStream(graph,'joint 2 velocity','deg/s',0,{0,1,0})
end

function sysCall_sensing()
    sim.setGraphStreamValue(graph,joint1Vel,180*sim.getJointVelocity(joint1Handle)/math.pi)
    sim.setGraphStreamValue(graph,joint1Vel,180*sim.getJointVelocity(joint1Handle)/math.pi)
end
```

```
function sysCall_init() 
    graph=sim.getObject('/Graph')
    joint1Vel=sim.addGraphStream(graph,'joint 1 velocity','deg/s',0,{1,0,0})
    joint2Vel=sim.addGraphStream(graph,'joint 2 velocity','deg/s',0,{0,1,0})

    joint1Handle=sim.getObject('./PioneerP3DX/leftMotor')
    joint1Handle=sim.getObject('./PioneerP3DX/rightMotor')
end

function sysCall_sensing()
    sim.setGraphStreamValue(graph,joint1Vel,180*sim.getJointVelocity(joint1Handle)/math.pi)
    sim.setGraphStreamValue(graph,joint1Vel,180*sim.getJointVelocity(joint1Handle)/math.pi)
end
```

First of all, I copied and pasted such code on the graph's script. Is it correct? I'm not certain, but I do think so.

The code is composed of two functions. As can be seen [here on the docs](https://www.coppeliarobotics.com/helpFiles/en/mainScript.htm), the `sysCall_init()` runs only when the simulation starts, while `sysCall_sensing()` runs once every simulation step.

The `sysCall_init()` instantiates a graph object and define two curves, named _joint 1 velocity_ and _joint 2 velocity_. The last thing it does is to retrive the joint handles associated with the motors. 

The `sysCall_sensing()` then updates the graph every step.

It looks simple and apparently it is supposed to be. But when I run it, the graph didn't change nor updated. :(

# 1st May Modifications 

How to make graphs work? Well, it is actually not complicated, _if you don't miss the difference between [child scripts](https://www.coppeliarobotics.com/helpFiles/en/childScripts.htm) and customization scripts_. I noticed it when I read [this forum question](https://forum.coppeliarobotics.com/viewtopic.php?t=9370). Whatever...

I added a graph in the scene tree: no big deal. The trick is adding a child script associated with the graph object. I chose a [Non-Threaded](https://www.coppeliarobotics.com/helpFiles/en/threadedAndNonThreadedCode.htm), Lua script. 

Once there, I can add the default graph script. I just needed to get the right Handlers (for which feature I wanted to graph) and _voilá_! Graphs served.

Apparently there are a difference between instantiating Streaming Graphs (i.e., time-dependent) and State-Space Graphs (i.e., global position). But there is no big deal in it: we just need to call different functions. These ones can be found on the [Graph Example](https://www.coppeliarobotics.com/helpFiles/en/graphs.htm).

The [Graph Example](https://www.coppeliarobotics.com/helpFiles/en/graphs.htm) was used as a demo for the State-Space Graph. I used the help file on [Object Orientation](https://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetObjectOrientation.htm) to get the robot's orientation and [this forum question](https://forum.coppeliarobotics.com/viewtopic.php?t=6070) to get the robot's position.