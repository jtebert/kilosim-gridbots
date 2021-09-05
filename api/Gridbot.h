#ifndef GRIDBOT_H
#define GRIDBOT_H

#include <kilosim/Robot.h>
#include <kilosim/Random.h>
#include <string>
#include <nlohmann/json.hpp>

#include "utils.h"

// for convenience
using json = nlohmann::json;

namespace Kilosim
{

    /*!
     * The abstract class Gridbot provides the implementation of functions in the
     * Gridbot API. This is designed to match what I already wrote for Gridsim
     */

    /*!
     * Gridbot message data (includes message and UID to avoid message duplication)
     */
    struct message_t
    {
        //! Message payload (string; should be max 128 bytes but I'm not checking now)
        json data;
        //! Unique ID for message, generated when message added to send queue
        uint32_t uid;
    };

    class Gridbot : public Robot
    {
    private:
        /*!
         * Buffer of the messages received by the communication protocol but not yet
         * processed by get_msg()
         */
        std::vector<message_t> m_msg_recv;

        //! x movement # of cells (set by move)
        double m_move_x_cells = 0;
        //! y movement # of cells (set by move)
        double m_move_y_cells = 0;

        //! Message to send to other robots (set by send_msg)
        json m_msg_send;

        //! Internal clock (in ticks); starts at 0
        unsigned int m_tick = 0;

        // PHYSICAL ROBOT PROPERTIES
        //! Radius of the robot (mm)
        double m_radius = 4.9; // Must be < m_grid_dim/2 (to avoid wall collisions at edges)
        // TODO: Check on/find out actual wheel distance/separation

        //! Dimensions of a grid cell (in mm)
        double m_grid_dim = 10;

    protected:
        //! Dimensions of the world in # of grid cells (instead of mm)
        int m_arena_grid_height;
        int m_arena_grid_width;

        //! Map of observed values in the environment. -1 means unobserved
        std::vector<std::vector<int>> m_map;

        //! Path to target, if used. Generated by utils/create_line()
        std::vector<Pos> m_path_to_target;
        bool m_follow_path = false;

    public:
        //! Communication range (mm)
        //! This is public so it can be configured by setup/simulation
        double comm_range = m_grid_dim * 2 * 8; //  8 bodylengths (default)

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
            message_t &rx_msg = *(static_cast<message_t *>(msg));
            uint32_t rx_uid = rx_msg.uid;
            // Check if the message is a duplicate by using its UID
            // for (unsigned int i = 0; i < m_msg_recv.size(); i++)
            // {
            // if (m_msg_recv[i].uid == rx_uid)
            // {
            //     return;
            // }
            // }
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
        Pos get_pos()
        {
            return xy_to_grid(x, y);
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
        std::vector<json> get_msg()
        {
            // Get the string message data from the incoming message buffer
            std::vector<json> str_msgs(m_msg_recv.size());
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
        void send_msg(json msg)
        {
            message_t out_msg;
            out_msg.data = msg;
            // Assign a random 16-bit integer to tag the message
            // Yeah, this isn't cryptographically secure or anything; it doesn't
            // need to be. It's a simulator-specific hack
            out_msg.uid = uniform_rand_int(0, INT32_MAX);
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

        /*! [Gridbot API] Sample cells around the robot's current position
         *
         * @return Colors in the current + neighboring 8 cells
         */
        std::map<Pos, double> sample_around()
        {
            // I'm lazy, so here's all 9 possible positions from a python output
            std::vector<Pos> pos_diff{
                {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 0}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
            std::map<Pos, double> samples;
            Pos grid_curr_pos = xy_to_grid(x, y);
            for (int i = 0; i < pos_diff.size(); i++)
            {
                Pos grid_pos_diff = {grid_curr_pos.x + pos_diff[i].x,
                                     grid_curr_pos.y + pos_diff[i].y};

                int sample_pos_x = x + pos_diff[i].x * m_grid_dim;
                int sample_pos_y = y + pos_diff[i].y * m_grid_dim;

                double sample_val;
                // Check if x or y is outside the arena bounds
                if (sample_pos_x < 0 || sample_pos_y < 0 || sample_pos_x > m_arena_width || sample_pos_y > m_arena_height)
                {
                    sample_val = -1;
                }
                else
                {
                    sample_val = m_light_pattern->get_ambientlight(
                        sample_pos_x,
                        sample_pos_y);
                }
                samples.insert({grid_pos_diff, sample_val});
            }
            // TODO: What happens if a sample is outside of the arena? -> segfault lol
            return samples;
        }

        // Convert (x,y) coordinates in mm to position in grid cells
        Pos xy_to_grid(double x, double y)
        {
            return {(int)((x - (m_grid_dim / 2)) / m_grid_dim),
                    (int)((y - (m_grid_dim / 2)) / m_grid_dim)};
        }

        // Convert cell/grid position to mm
        // TODO: Make this return doubles?
        Pos grid_to_xy(Pos p)
        {
            return {(p.x * m_grid_dim) + (float)m_grid_dim / 2,
                    (p.y * m_grid_dim) + (float)m_grid_dim / 2};
        }

    public:
        void set_path(const int x0, const int y0, int x1, int y1)
        {
            // This takes care of an edge case where it tries to generate paths
            // longer than the max length of a C++ vector......
            double dist = sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2));
            double max_dist = 1000;
            if (dist > max_dist)
            {
                // This is too long; cut it down to the max length
                if (x1 - x0 != 0)
                {
                    x1 = ((x1 - x0) * max_dist / dist) + x0;
                }
                if (y1 - y0 != 0)
                {
                    y1 = ((y1 - y0) * max_dist / dist) + y0;
                }
            }
            std::vector<Pos> path = create_line(x0, y0, x1, y1);
            m_path_to_target.resize(path.size());
            m_path_to_target = path;
            m_follow_path = true;
            // When a path is set, turn off other movement
            move(0, 0);
        }
        /*!
         * Override the Robot pseudophysics to move on a grid
         */
        RobotPose robot_compute_next_step() override
        {
            // Use the current position
            // This function does NOT do the collision checking
            // This will use the computed m_path_to_target (if set)
            // If not set, it computes the move from m_move_x_cells and m_move_y_cells

            double new_x, new_y;

            if (m_follow_path && m_path_to_target.size() > 0)
            {
                // Follow the next position in the pre-computed path
                Pos new_pos = m_path_to_target.back();
                // Pos curr_pos = get_pos();
                // int x_diff = curr_pos.x - new_pos.x;
                // int y_diff = curr_pos.y - new_pos.y;
                // if (x_diff > 1 || x_diff < -1 || y_diff > 1 || y_diff < -1)
                // {
                //     printf("INVALID POSITION CHANGE:\n");
                //     //     std::cout << "(" << curr_pos.x << "," << curr_pos.y << ") -> (" << new_pos.x << "," << new_pos.y << ")" << std::endl;
                // }
                // std::cout << new_pos.x << ", " << new_pos.y << std::endl;
                new_x = new_pos.x * m_grid_dim + m_grid_dim / 2;
                new_y = new_pos.y * m_grid_dim + m_grid_dim / 2;
                m_path_to_target.pop_back();
                if (m_path_to_target.size() == 0)
                {
                    m_follow_path = false;
                }
            }
            else
            {
                // Convert movement in cells to movement in screen/IRL space
                new_x = x + m_move_x_cells * m_grid_dim;
                new_y = y + m_move_y_cells * m_grid_dim;
            }

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
            // TODO: Add check for whether robot is trying to move further than neighboring cell
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
        void robot_init(double x0, double y0, double theta0) override
        {
            Pos init_pos = grid_to_xy({x0, y0});
            x = init_pos.x;
            y = init_pos.y;
            theta0 = 0.0;

            // Assign unique ID
            id = uniform_rand_int(0, 2147483640);

            // Get dimensions of arena in grid cells (instead of mm)
            m_arena_grid_width = m_arena_width / m_grid_dim;
            m_arena_grid_height = m_arena_height / m_grid_dim;

            // Initialize map
            //vector<vector<int>> M;
            //int m = number of rows, n = number of columns;
            // https://stackoverflow.com/a/26006446/2552873
            m_map.resize(m_arena_grid_height, std::vector<int>(m_arena_grid_width, -1));

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

#endif
