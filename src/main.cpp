#include "DemoGridbot.cpp"
#include <kilosim/World.h>
#include <kilosim/Viewer.h>

int main(int argc, char *argv[])
{
    // Create 3m x 3m world (no background image, for now)
    Kilosim::World world(
        3000, 3000);

    Kilosim::Viewer viewer(world);

    int num_robots = 10;

    // Create robots and place in world
    std::vector<Kilosim::DemoGridbot *> robots(num_robots);
    for (int n = 0; n < num_robots; n++)
    {
        robots[n] = new Kilosim::DemoGridbot();
        world.add_robot(robots[n]);
        robots[n]->robot_init((n + 1) * 200, (n + 1) * 200, 1.7);
    }

    // Verify that robots are within World bounds and not overlapping
    world.check_validity();

    double trial_duration = 300; // seconds
    while (world.get_time() < trial_duration)
    {
        // printf("stepping\n");
        world.step();
        viewer.draw();
    }

    printf("Finished");

    return 0;
}