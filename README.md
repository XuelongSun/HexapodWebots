# Build A Hexapod Robot in Webots

A Webots project for a hexapod robot featured with `Forward Kinematics`, `Inverse Kinematics` and `Gait Control`

![](media/demo.gif)

### Tutorial
A [video tutorial](https://www.bilibili.com/video/BV1EF411U7bW) for building this project from scratch is available from [my channel](https://space.bilibili.com/13031745) on Bilibili (Chinese Language).

### Demos

Change the `controllers/hexapod.py` file to run different demos. For example, walking with IK generated tripod gait:

```python
    robot = Hexapod()
    robot.generate_walking_sequence_ik({
        'Gait':'Tripod',
        'Length': 0.04,
        'Height': 0.06,
        'StepNum':20
    })
    
    robot.walk(robot.walking_sequence_ik)
```
