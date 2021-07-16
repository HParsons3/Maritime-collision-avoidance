# Maritime-collision-avoidance

Currently, the system only has one view mode, but I intend to add more

Run runSim.m. The number of entities(ships) can be modified, as well as the size of the map, via "entitycount" and "mapsize" respectively. I found it useful to have enough entities to cause at least one collision per map useful, but too many can cause overcrowding which can make the view difficult to read. 

Runsim will pause after the map is generated. In this map there will be an assortment of diamonds (ship starting positions) and circles (Ship goals). As of right now, they are all set to the same colour. 

After clicking continue, the simulation will continue to pause after every tick, so the user can view what is happening every frame. The current planned paths will be displayed as a series of points leading to the goal. Each entity will also form a line, showing the location at the beginning of the tick and at the end. If any of these lines cross, it implies a collision MAY happen in that tick. Therefore, path planning begins. This will be made obvious by the path of one (or both) of the vessels deforming into a collision avoidance path. Continuing the simulation after this point will show the vessel(s) who's path has changed will follow the new path rather than the old. The simulation will exit once all vessels reach their destination (Currently this throws an error which will be patched out soon).

Bugs: Display is hard to read sometimes, especially with a lot of vessels. I plan to add several different display types which can be opted in and out of in runSim.m

Sometimes ships "jump" long distances after path correction. Will be patched out soon.

目前，系统只有一种视图模式，但我打算添加更多

运行 runSim.m。 物体（船）的数量和地图的大小可以分别通过“entitycount”和“mapsize”进行修改。我发现有足够的物体使每个地图至少发生一次的碰撞会很有效，但太多会导致过度拥挤，这会使视图难以阅读。

Runsim 会在地图生成后暂停。 在这张地图中，会有各种各样的钻石（船舶起始位置）和圆圈（船舶目标）。 截至目前，它们都设置为相同的颜色。

点击继续后，模拟将在每一个步骤后停止，以便用户可以查看每一帧发生的情况。 当前计划的路径将显示为一系列通向目标的点。 每个物体也将形成一条线，显示每一步骤开始和结束的位置。 如果这些线中的任何一条交叉，则意味着在那个步中可能发生碰撞。 当路径规划开始时，我们能够更加清楚地看到，通过一条（或两条）直接路径变形成为的避免碰撞路径。 在这一点后，继续进行模拟将展示出：路径发生变化的船舶将只遵循新路径而不是旧路径。 一旦所有船只到达目的地，模拟将退出（目前这会引发一个错误，将很快修复）。

错误：有时显示很难阅读，尤其是在有很多船只的情况下。 我计划添加几种不同的显示类型，可以在 runSim.m 中选择加入和退出

有时船舶在路径修正后“跳跃”很长的距离。 很快就会被修补。
