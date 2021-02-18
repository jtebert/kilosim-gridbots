#include "Gridbot.h"

namespace Kilosim
{
    class DemoGridbot : public Gridbot
    {
        int m_test = 0;

        void setup()
        {
            move(1, 1);
            set_led(100, 100, 100);
        };

        void loop()
        {
            if (m_test <= 6)
            {
                std::cout << "\n"
                          << m_test << std::endl;
                std::map<Pos, double> samples = sample_around();
                std::cout << samples.size() << std::endl;
                for (auto const &x : samples)
                {
                    std::cout << x.first.x << ", " << x.first.y // string (key)
                              << ": "
                              << x.second // string's value
                              << std::endl;
                }
            }
            m_test++;
        };
    };
} // namespace Kilosim
