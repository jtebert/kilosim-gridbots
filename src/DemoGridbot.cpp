#include "Gridbot.h"
#include "utils.h"

namespace Kilosim
{
    class DemoGridbot : public Gridbot
    {
        int m_test = 0;

        void setup()
        {
            // move(1, 1);
            set_led(100, 0, 0);

            // Try going in a straight line from utils.create_line (Gridbot.gen_line)
            set_path(0, 0, 80, 50);
        };

        void loop()
        {
            if (m_test <= 8)
            {
                // std::cout << "\n"
                //           << m_test << std::endl;
                std::map<Pos, double> samples = sample_around();
                // std::cout << samples.size() << std::endl;
                for (auto const &s : samples)
                {
                    // std::cout << s.first.x << ", " << s.first.y // string (key)
                    //           << ": "
                    //           << s.second // string's value
                    //           << std::endl;
                }
            }
            m_test++;
        };
    };
} // namespace Kilosim
