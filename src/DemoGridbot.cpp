#include "Gridbot.h"

namespace Kilosim
{
    class DemoGridbot : public Gridbot
    {

        void setup()
        {
            move(1, 1);
            set_led(100, 100, 100);
        };

        void loop(){};
    };
} // namespace Kilosim
