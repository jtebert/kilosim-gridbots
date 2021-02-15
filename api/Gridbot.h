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

        //! Message to send to other robots (set by send_msg)
        std::string m_msg_send;

        //! Right motor speed (set by drive_robot)
        double m_motor_speed_r = 0;
        //! Left motor speed (set by drive_robot)
        double m_motor_speed_l = 0;
        //! Conversion from PWM to speed (100% PWM = this mm/s) - left motor
        double m_motor_rate_l = 500;
        //! Conversion from PWM to speed (100% PWM = this mm/s) - right motor
        double m_motor_rate_r = 500;
        // TODO: Figure out actual conversion from PWM to speed (this also assumes its linear)
        // TODO: Add some noise to motor rates in initialization

        //! Internal clock (in ticks); starts at 0
        unsigned int m_tick = 0;

        // PHYSICAL ROBOT PROPERTIES
        //! Distance between the wheel centers (mm)
        double m_wheel_dist = 100; // 10 cm
        //! Radius of the robot (mm)
        double m_radius = 60; // 60 mm (12 cm diameter)
        // TODO: Check on/find out actual wheel distance/separation

    protected:
        // TODO: Fill in any protected variables as needed

    public:
        //! Communication range (mm)
        //! This is public so it can be configured by setup/simulation
        double comm_range = 12 * 10 * 5; //  5 bodylengths (default)

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
         * COACHBOT/PISWARM API FUNCTIONS
         **************************************************************************/

        /*!
         * [Coachbot API] Returns the robot's current orientation and x-y position
         *
         * Returns a 3-element vector (x, y, theta), which is the robot's x-y position and
         * orientation theta in a global frame. The unit of `x` and `y` are robot's
         * radius, specifically, x = 1 y = 0 means the robot is 2.5 radius away from
         * the origin, and the unit of `theta` is radians.
         * @return Vector of the robot's (x, y, theta position)
         */
        std::vector<double> get_pose()
        {
            return {x, y, theta};
        }

        /*!
         * [Coachbot API] Returns the messages received since last time the function
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
         * [Coachbot API] Transmits the message `msg` to the other robots
         *
         * The input should be a string with a length of 128 bytes (This length can
         * be changed if it is necessary).
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
         * [Coachbot API] Send the speeds and directions of the robot's wheels
         *
         * the `l` and `r` should be integers ranging from -100 to 100. The absolute
         * value is the PWM duty cycle percentage, and the sign controls the
         * direction of the wheel.
         *
         * @param l PWM percentage of the left wheel (-100 to 100; sign is direction)
         * @param r PWM percentage of the right wheel (-100 to 100; sign is direction)
         */
        void drive_robot(double l, double r)
        {
            m_motor_speed_l = l;
            m_motor_speed_r = r;
            // Convert these into forward_speed and turn_speed to match Robot/run_controller
        }

        /*!
         * [Coachbot API] Configure the color and brightness of the robot's LED
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
        double get_clock()
        {
            // Matches the way this is handled by World
            // (Stored internally as integer to avoid floating point errors)
            return (double)m_tick * m_tick_delta_t;
        }

    public:
        /*!
         * Override the Robot pseudophysics to simulate the Coachbot differential drive
         */
        RobotPose robot_compute_next_step() const
        {
            // Use the current pose and the motor speeds
            // This function does NOT do the collision checking
            // Assume theta is 0 when pointing along x axis
            // TODO: Check directions of axes (might be off/backwards from display)

            // Uses the differential drive kinematics from here:
            // https://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf

            // 0) Convert wheel PWM to speeds
            double v_l = m_motor_speed_l * m_motor_rate_l / 100;
            double v_r = m_motor_speed_r * m_motor_rate_r / 100;

            double new_x, new_y, new_theta;

            if (v_l == v_r)
            {
                // Special case where robot is going straight. If you put this
                // through the full differential drive kinematics, you get 0 in the
                // denominator.
                double dx = v_l * cos(theta) * m_tick_delta_t;
                double dy = v_l * sin(theta) * m_tick_delta_t;
                new_x = x + dx;
                new_y = y + dy;
                new_theta = theta; // Same angle because going straight
            }
            else
            {
                // 1) Compute R - distance between robot center and ICC
                //    R = l/2 * (v_l+v_r)/(v_r-v_l), where l is distance between wheels
                double R = (m_wheel_dist / 2) *
                           ((v_l + v_r) / (v_r - v_l));

                // 2) Compute the velocity around the ICC
                //    omega = (v_r-v_l)/l
                double omega = (v_r - v_l) / m_wheel_dist;

                // 3) Compute the ICC (instantaneous center of curvature)
                //    ICC_x = x - R*sin(theta)
                //    ICC_y = y + R*cos(theta)
                double icc_x = x - (R * sin(theta));
                double icc_y = y + (R * cos(theta));

                // 4) Compute new x, y, theta from the above
                //    x' = cos(omega*dt) * (x-ICC_x) + -sin(omega*dt) * (y-ICC_y) + ICC_x
                //    y' = sin(omega*dt) * (x-ICC_x) +  cos(omega*dt) * (y-ICC_y) + ICC_y
                //    theta' = omega * dt + theta
                double omega_dt = omega * m_tick_delta_t;
                new_x = (cos(omega_dt) * (x - icc_x)) - (sin(omega_dt) * (y - icc_y)) + icc_x;
                new_y = (sin(omega_dt) * (x - icc_x)) + (cos(omega_dt) * (y - icc_y)) + icc_y;
                new_theta = omega_dt + theta;
            }

            // std::cout << id << ": " << new_x << ", " << new_y << ", " << new_theta << std::endl;

            return {new_x, new_y, wrap_angle(new_theta)};
        }

        // IDK what this does but it won't compile without it
        char *get_debug_info(char *buffer, char *rt)
        {
            return buffer;
        }
    };

} // namespace Kilosim