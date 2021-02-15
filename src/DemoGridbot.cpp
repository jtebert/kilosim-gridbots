#include "Gridbot.h"

namespace Kilosim
{
    class DemoGridbot : public Gridbot
    {

        void setup()
        {
            drive_robot(50, 40);
            set_led(100, 100, 100);
        };

        void loop(){};
    };
} // namespace Kilosim
