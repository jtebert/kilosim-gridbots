#include <kilosim/Robot.h>
#include <kilosim/Random.h>
#include <string>

namespace Kilosim
{

    /*!
     * The abstract class Gridbot provides the implementation of functions in the
     * Gridbot API. This is designed to match what I already wrote for Gridsim
     */

    /*!
     * Gridbot message data (includes message and UID to avoid message duplication)
     */
    struct cb_message_t
    {
        //! Message payload (string; should be max 128 bytes but I'm not checking now)
        std::string data;
        //! Unique ID for message, generated when message added to send queue
        uint32_t uid;
    };

    class Gridbot : public Robot
    {
    private:
        // TODO: Fill in any private variables as needed
        /*!
         * Buffer of the messages received by the communication protocol but not yet
         * processed by get_msg()\
         */
        std::vector<cb_message_t> m_msg_recv;

        //! x movement # of cells (set by move)
        double m_move_x_cells = 0;
        //! y movement # of cells (set by move)
        double m_move_y_cells = 0;

        //! Message to send to other robots (set by send_msg)
        std::string m_msg_send;

        //! Internal clock (in ticks); starts at 0
        unsigned int m_tick = 0;

        // PHYSICAL ROBOT PROPERTIES
        //! Radius of the robot (mm)
        double m_radius = 9;
        // TODO: Check on/find out actual wheel distance/separation

        //! Dimensions of a grid cell (in mm)
        double m_grid_dim = 10;

    protected:
        // TODO: Fill in any protected variables as needed

    public:
        //! Communication range (mm)
        //! This is public so it can be configured by setup/simulation
        double comm_range = m_grid_dim * 2 * 8; //  8 bodylengths (default)

        //! Position in grid cells. This is what you should use for logging
        int m_grid_x;
        int m_grid_y;

    private:
        /*!
         * Get a void pointer to the message the robot is sending and handle any
         * callbacks for successful message transmission
         * @return Pointer to message to transmit
         */
        void *get_tx_message()
        {
            // return * m_msg_send;
            return static_cast<void *>(&m_msg_send);
        }

        /*!
         * Determine if another robot is within communication range
         * This is called by a transmitting (tx) robot to verify if the receiver is
         * within range when sending a message OUT. Because of possible
         * asymmetries in communication range, both comm_criteria() must be met by
         * both the tx and rx robots for a message to be successfully transmitted.
         * @param dist Distance between the robots (in mm)
         * @return true if robot can communicate with another robot
         */
        bool comm_criteria(double dist)
        {
            // Check if within communication range only
            return dist <= comm_range;
        }

        /*!
         * This is called by a transmitting robot (tx) to set a flag for calling the
         * message success callback (message_tx_success)
         */
        void received()
        {
            // TODO: Not sure if I need this (called by sender when message successfully sent`)
        }

        /*!
         * This is called when a robot (rx) receives a message. It calls some
         * message handling function (e.g., message_rx) specific to the
         * implementation.
         */
        void receive_msg(void *msg, double dist)
        {
            cb_message_t &rx_msg = *(static_cast<cb_message_t *>(msg));
            uint32_t rx_uid = rx_msg.uid;
            // Check if the message is a duplicate by using its UID
            for (unsigned int i = 0; i < m_msg_recv.size(); i++)
            {
                if (m_msg_recv[i].uid == rx_uid)
                {
                    return;
                }
            }
            // Add the message to the buffer if UID not found
            m_msg_recv.push_back(rx_msg);
        }

        /*!
         * Perform any one-time initialization for the specific implementation of
         * the Robot, such as setting initial battery levels and calling any
         * user-implementation setup functions. It is called by `robot_init()`.
         *
         * If you want to change the robot's battery life, do so here by setting
         * the `battery` member variable.
         */
        void init()
        {
            // TODO: Not sure what needs to be initialized? Battery? Motor rates?

            // Run the user's setup function
            setup();
        }

        /*!
         * Internal control loop for the specific Robot subclass implementation.
         * This performs any robot-specific controls such as setting motors,
         * communication flags, and calling user implementation loop functions.
         * It is called every simulation time step by `robot_controller()`
         */
        void controller()
        {
            // Increment clock (needs tick rate)
            m_tick++;
            // Run user loop code
            loop();
            // Run motor/movement code
            // Possibly set message transmission flags?
        }

        double get_radius() const
        {
            return m_radius;
        }

    protected:
        /***************************************************************************
	     * REQUIRED USER API FUNCTIONS
	     **************************************************************************/

        /*!
         * [User API] User-implemented setup function that is run once in
         * initialization
         */
        virtual void setup() = 0;

        /*!
         * [User API] User-implemented loop function that is called for the Coachbot
         * on every loop
         */
        virtual void loop() = 0;

        /***************************************************************************
         * GRIDBOT API FUNCTIONS
         **************************************************************************/

        /*!
         * [Gridbot API] Returns the robot's current x-y position
         *
         * Returns a 2-element vector (x, y), which is the robot's x-y position in a global frame.
         * The unit of `x` and `y` are robot's radius, specifically, x = 1 y = 0 means the robot is
         * 2.5 radii away from the origin.
         * (These values come from Robot.h)
         * @return Vector of the robot's (x, y position)
         */
        std::vector<double> get_pos()
        {
            return {x, y};
        }

        /*!
         * [Gridbot API] Returns the messages received since last time the function
         * was called
         *
         * Returns the messages received from the other robots, then cleans the
         * shared memory `msg_recv`
         *
         * @return Vector of all the message strings
         */
        std::vector<std::string> get_msg()
        {
            // Get the string message data from the incoming message buffer
            std::vector<std::string> str_msgs(m_msg_recv.size());
            for (unsigned int i = 0; i < m_msg_recv.size(); i++)
            {
                str_msgs[i] = m_msg_recv[i].data;
            }
            // Clear message queue
            m_msg_recv.clear();
            return str_msgs;
        }

        /*!
         * [Gridbot API] Transmits the message `msg` to the other robots
         *
         * The input should be a string of (currently) arbitrary length.
         *
         * @param msg Message to send to all other robots within communication range
         */
        void send_msg(std::string msg)
        {
            cb_message_t out_msg;
            out_msg.data = msg;
            // Assign a random 32-bit integer to tag the message
            // Yeah, this isn't cryptographically secure or anything; it doesn't
            // need to be. It's a simulator-specific hack
            out_msg.uid = uniform_rand_int(0, UINT32_MAX);
            m_msg_send = msg;
        }

        /*!
         * [Gridbot API] Number of cells to move in x and y
         *
         * @param int Number of cells to move in x direction
         * @param int Number of cells to move in y direction
         */
        void move(int x, int y)
        {
            m_move_x_cells = x;
            m_move_y_cells = y;
        }

        /*!
         * [Gridbot API] Configure the color and brightness of the robot's LED
         *
         * `r`, `g`, `b`, should be integers ranging from 0 to 100. Each number
         * controls the intensity of the corresponding channel of the LED.
         *
         * @param r Intensity or red RGB LED channel (0-100)
         * @param g Intensity or green RGB LED channel (0-100)
         * @param b Intensity or blue RGB LED channel (0-100)
         */
        void set_led(unsigned int r, unsigned int g, unsigned int b)
        {
            // Internal Robot color uses doubles in range 0-1 (scale by 100)
            color[0] = (double)r / 100;
            color[1] = (double)g / 100;
            color[2] = (double)b / 100;
        }

        /*!
         * [Gridbot API] Return time elapsed since the code started
         *
         * @return Seconds since the code started
         */
        double get_tick()
        {
            // Matches the way this is handled by World
            return m_tick;
        }

    public:
        /*!
         * Override the Robot pseudophysics to move on a grid
         */
        RobotPose robot_compute_next_step() const override
        {
            // Use the current position
            // This function does NOT do the collision checking

            double new_x, new_y;

            // Convert movement in cells to movement in screen/IRL space
            new_x = x + m_move_x_cells * m_grid_dim * 2;
            new_y = y + m_move_y_cells * m_grid_dim * 2;

            // std::cout << id << ": " << new_x << ", " << new_y << std::endl;

            // We're not using angle, so just always set it to 0.
            return {new_x, new_y, 0};
        }

        /*!
         * Override the Robot movement based on collisions.
         * In this case, we IGNORE any collisions with other robots (ie they're allowed to intersect)
         * but we still prevent moving outside of the arena.
         */
        void robot_move(const RobotPose &new_pose, const int16_t &collision) override
        {
            if (collision != -1)
            {
                // Execute move whether there's a collision or not
                x = new_pose.x;
                y = new_pose.y;
            }
            // Otherwise it's a wall collision. In that case, just stay put
        }

        /*!
         * Override the default robot_init to place the robot on a grid position.
         * Here, x0 and y0 are now GRID positions
         */
        void robot_init(double x0, double y0, double theta0)
        {
            x = (x0 * m_grid_dim * 2) + m_grid_dim;
            y = (y0 * m_grid_dim * 2) + m_grid_dim;
            theta0 = 0.0;

            m_grid_x = x0;
            m_grid_y = y0;

            // Assign unique ID
            id = uniform_rand_int(0, 2147483640);

            // Run implementation-specific initialization
            init();
        }

        // IDK what this does but it won't compile without it
        char *get_debug_info(char *buffer, char *rt)
        {
            return buffer;
        }
    };

} // namespace Kilosim