RunsimV2.m

Run runSimV2.m. The starting locations, goal locations and speed of the ships can be modified to force certain types of collisions.

The code will pause after the first paths are generated, displaying on the map two diamonds (Starting locations) and two rings (Goal locations). A line of dots will display the currently predicted paths.

After clicking continue, the system will step forwards every time the button is pressed, showing a step-by-step display of the ships' movement. In each step, the start of the line is where the ship is at the beginning of that time frame, and the end of the line is where the ship will be at the end of the time frame.

When a possible collision is detected (The lines between the start and end point for the two ships in a single step cross each other), one of the paths will change, following the rules of COLREGs. If ship A has to change path, the line from the current position to the first point of the corrected path will be invisible due to the order the system draws the graph in. 

Runsim.m:

Run runSim.m. The number of entities(ships) can be modified, as well as the size of the map, via "entitycount" and "mapsize" respectively. I found it useful to have enough entities to cause at least one collision per map useful, but too many can cause overcrowding which can make the view difficult to read.

Runsim will pause after the map is generated. In this map there will be an assortment of diamonds (ship starting positions) and circles (Ship goals). As of right now, they are all set to the same colour.

After clicking continue, the simulation will continue to pause after every step, so the user can view what is happening every frame. The current planned paths will be displayed as a series of points leading to the goal. Each entity will also form a line, showing the location at the beginning of the step and at the end. If any of these lines cross, it implies a collision MAY happen in that step. Therefore, path planning begins. This will be made obvious by the path of one (or both) of the vessels deforming into a collision avoidance path. Continuing the simulation after this point will show the vessel(s) who's path has changed will follow the new path rather than the old. The simulation will exit once all vessels reach their destination.

RunsimV2.m

运行 runSimV2.m。可以修改船只的起始位置、目标位置和速度以强制某些类型的碰撞。

生成第一条路径后，代码将暂停，在地图上显示两个菱形（起始位置）和两个环（目标位置）。一行点将显示当前预测的路径。

点击continue后，每按一次按钮，系统就会向前走一步，一步步显示船舶的运动情况。在每一步中，线的起点是船舶在该时间范围开始时的位置，线的终点是船舶在时间范围结束时所在的位置。

当检测到可能的碰撞时（两艘船的起点和终点之间的线在一个步骤中相互交叉），其中一条路径将按照 COLREG 的规则发生变化。如果船舶 A 必须改变路径，由于系统绘制图形的顺序，从当前位置到校正路径的第一个点的线将不可见。

Runsim.m:

运行 runSim.m。可以分别通过“entitycount”和“mapsize”修改实体（船舶）的数量以及地图的大小。我发现有足够的实体来导致每张地图至少发生一次碰撞很有用，但太多会导致过度拥挤，这会使视图难以阅读。

Runsim 会在地图生成后暂停。在这张地图中，会有各种各样的钻石（船舶起始位置）和圆圈（船舶目标）。截至目前，它们都设置为相同的颜色。

点击继续后，模拟会在每一步后继续暂停，以便用户可以查看每一帧发生的情况。当前计划的路径将显示为一系列通向目标的点。每个实体也将形成一条线，显示步骤开始和结束时的位置。如果这些线中的任何一条交叉，则意味着在该步骤中可能发生碰撞。因此，路径规划开始。这将通过一个（或两个）容器的路径变形为防撞路径而变得明显。在这一点之后继续模拟将显示路径发生变化的船只将遵循新路径而不是旧路径。一旦所有船只到达目的地，模拟将退出。


