# Maritime-collision-avoidance

Currently, the system only has one view mode, but I intend to add more:

Run runSim.m. The number of entities(ships) can be modified, as well as the size of the map, via "entitycount" and "mapsize" respectively. I found it useful to have enough entities to cause at least one collision per map useful, but too many can cause overcrowding which can make the view difficult to read. 

Runsim will pause after the map is generated. In this map there will be an assortment of diamonds (ship starting positions) and circles (Ship goals). As of right now, they are all set to the same colour. 

After clicking continue, the simulation will continue to pause after every tick, so the user can view what is happening every frame. The current planned paths will be displayed as a series of points leading to the goal. Each entity will also form a line, showing the location at the beginning of the tick and at the end. If any of these lines cross, it implies a collision MAY happen in that tick. Therefore, path planning begins. This will be made obvious by the path of one (or both) of the vessels deforming into a collision avoidance path. Continuing the simulation after this point will show the vessel(s) who's path has changed will follow the new path rather than the old. The simulation will exit once all vessels reach their destination (Currently this throws an error which will be patched out soon).

Bugs: Display is hard to read sometimes, especially with a lot of vessels. I plan to add several different display types which can be opted in and out of in runSim.m

Sometimes ships "jump" long distances after path correction. Will be patched out soon.
