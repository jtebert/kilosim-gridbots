#include "DemoGridbot.cpp"
#include <kilosim/World.h>
#include <kilosim/Viewer.h>

#include <unistd.h>

int main(int argc, char *argv[])
{
    // Create 3m x 3m world (no background image, for now)
    Kilosim::World world(
        3000, 3000);

    Kilosim::Viewer viewer(world, 1200);

    int num_robots = 1;

    // Create robots and place in world
    std::vector<Kilosim::DemoGridbot *> robots(num_robots);
    for (int n = 0; n < num_robots; n++)
    {
        robots[n] = new Kilosim::DemoGridbot();
        world.add_robot(robots[n]);
        robots[n]->robot_init(0, 0, 0);
    }

    std::vector<Pos> path = robots[0]->gen_line(10, 10, 12, 5);
    // for (int i = 0; i < path.size(); i++)
    // {
    //     std::cout << path[i].x << ", " << path[i].y << std::endl;
    // }

    // Verify that robots are within World bounds and not overlapping
    // world.check_validity();

    double trial_duration = 300; // seconds
    sleep(2);
    while (world.get_time() < trial_duration)
    {
        // printf("stepping\n");
        world.step();
        viewer.draw();
        sleep(.25);
    }

    printf("Finished");

    return 0;
}