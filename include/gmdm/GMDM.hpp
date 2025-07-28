// Copyright (c) 2025 LINKS Lab UConn. All rights reserved.
// SPDX-License-Identifier: MIT
// Author: James P. Wilson (james.wilson@uconn.edu)


#ifndef GMDM_HPP
#define GMDM_HPP

#include <vector>
#include <iostream>
#include <cassert>

namespace gmdm
{

    // Constants for different path types
    const int CSC_PATH_TYPE = 1;
    const int CCC_PATH_TYPE = 2;
    const int OTHER_PATH_TYPE = 0;
    constexpr double PI = 3.14159265358979323846;

    /* @brief SE2 pose is used to specify the start and end points of a path */
    struct PoseSE2
    {
        double x;     // in meters (without loss of generality)
        double y;     // in meters (without loss of generality)
        double theta; // in radians
    };

    /* @brief A GMDM state is an SE2 Pose with extra information (useful for when embedded into a path) - linear speed, turning rate, and time stamp 
     * it is called a state for simplicity with some abuse of terminology
    */
    struct GMDMState
    {
        PoseSE2 pose;
        double v;     // linear speed between v_min and v_max
        double omega; // turning rate radians / sec between -omega_max and omega_max. a negative value means on a right turn. a positive value means a left turn. 0 means going straight.
        double t;     // time stamp in seconds
    };

    /* @brief Each segment is defined by constant linear speed, turning rate, and time duration. Note: the turning radius is v/|omega|
       Note: a nonnegative time duration tau (>=0) is only calculated when solving a path.
       Note: when tau >= 0 and finite, this is a properly defined "motion primitive".
       Note: if tau = infinity, that means a feasible path  does not exist for the given problem
       If tau <= -1, the path was not solved.
    */
    struct GMDMSegment
    {
        double v;         // linear speed
        double omega;     // turn rate
        double tau = -1.; // duration
    };

    /* @brief Each GMDM path is made of three GMDM segments
     * GMDM paths can only be of type CSC (e.g., seg1.omega != 0 and seg2.omega == 0 and seg3.omega != 0)
     *       or of type CCC (e.g., sign(seg1.omega) != sign(seg2.omega) and sign(seg2.omega) != sign(seg3.omega) and seg.omega != 0 for all segments)
     * The above is asserted in relevant functions
     * Note if travel_time() < 0, then the path was not solved
     * Note if travel_time() = infinity, then the path is not feasible for the given problem
     */
    struct GMDMPath
    {
        GMDMSegment seg1;
        GMDMSegment seg2;
        GMDMSegment seg3;

        double travel_time() const
        {
            return seg1.tau + seg2.tau + seg3.tau;
        }
    };

    /*
     * @class GMDM
     * @brief The main class for GMDM with the key functionality needed for solving for and generating paths
     */

    class GMDM
    {
    public:
        /* GMDM Constructor\
         * @param v_min: minimum linear speed (m/s without loss of generality). Must be greater than 0
         * @param v_max: maximum linear speed (m/s without loss of generality). Must be greater than v_min
         * @param omega_max: maximum turning rate (radians/sec). Must be greater than 0
         * @param num_v: number of linear speeds to consider for each segment when evaluation GMDM paths. Recommend num_v = 2 for time-optimal planning. If num_v = 1, GMDM becomes Dubins, and v_max is used as the constant speed.
         * @param use_v_max_on_straight: bool to determine if on CSC path types, the speed on the S (straight) segment is always vmax. This is useful to minimize computation time when minimizing travel time is the only consideration. For planning considering risk/safety, it is recommended to disable this feature.
         * @param num_omega: number of (positive) turning rates to consider for each turning segment when evaluating GMDM paths (e.g., if num_omega = 1, then the set of turning rates considered is [-omega_max, 0, omega_max]). Recommend num_omega = 1 for most problems.
         */
        GMDM(double v_min, double v_max, double omega_max, int num_v = 2, bool use_v_max_on_straight = true, int num_omega = 1);

        // getters definitions

        double get_v_min() { return v_min_; }
        double get_v_max() { return v_max_; }
        double get_omega_max() { return omega_max_; }
        double get_num_v() { return num_v_; }
        double get_num_omega() { return num_omega_; }
        bool get_use_v_max_on_straight() { return use_v_max_on_straight_; }
        bool get_is_solved() { return is_solved_; }
        PoseSE2 get_start_pose() { return pose_start_; }
        PoseSE2 get_goal_pose() { return pose_goal_; }

        /* @brief returns the set of linear speeds considered by GMDM */
        const std::vector<double> &get_v_set() { return v_set_; }

        /* @brief returns the set of turning rates considered by GMDM */
        const std::vector<double> &get_omega_set() { return omega_set_; }

        /* @brief returns all path types considered by GMDM */
        const std::vector<GMDMPath> &get_all_path_types() { return all_path_types_; }

        /* @brief returns the ranks of the paths if the problem has been solved.*/
        const std::vector<size_t> &get_path_ranks()
        {
            assert(is_solved_ = true);
            return rank_;
        }

        /* @brief returns the fastest GMDM path if the problem has been solved */
        const GMDMPath &get_fastest_path()
        {
            assert(is_solved_ == true);
            return all_path_types_[rank_[0]];
        }

        /* @brief returns the GMDM path of the specified rank. */
        const GMDMPath &get_path_of_specific_rank(size_t k)
        {
            assert(is_solved_ == true);
            if (k >= all_path_types_.size())
                k = all_path_types_.size() - 1;
            return all_path_types_[rank_[k]];
        }

        // setters

        /* @brief sets the min. linear speed. Changes the overall set of speeds considered. Must call reset() after setting. */
        void set_v_min(double v_min);

        /* @brief sets the max. linear speeds. Changes the overall set of speeds considered. Must call reset() after setting. */
        void set_v_max(double v_max);

        /* @brief sets the max turning rate (rad/sec). Changes the overall set of turning rates considered. Must call reset() after setting */
        void set_omega_max(double omega_max);

        /* @brief changes the number of speeds considered between v_min and v_max. Must call reset() after setting */
        void set_num_v(int num_v);

        /* @brief changes the number of (positive) turning rates considered. Must call reset() after setting. */
        void set_num_omega(int num_omega);

        /* @brief determines if a straight line segment should always be at max speed or not. Must call reset() after setting.*/
        void set_use_v_max_on_straight(bool use_v_max_on_straight);

        /* @brief define the start pose of the GMDM path */
        void set_start_pose(const PoseSE2 &start);

        /* @brief define the end pose of the GMDM path */
        void set_goal_pose(const PoseSE2 &goal);

        /* @brief set the spacing (in time) between states when interpolating a path */
        void set_time_step(double time_step);

        // key functions

        /* @brief solves the overall planning problem.
         * Calls solve_one(...) as a subroutine on each GMDM path type.
         * Each path is then ranked according to the resulting travel time.
         */
        void solve_all();

        /* @brief solves the planning problem for the specified GMDM path type.
         * Basically, finds the time durations tau1, tau2, and tau3 of each segment for the defined start and goal poses.
         */
        void solve_one(GMDMPath &path);

        /* @brief determines if a particular GMDMPath is valid */
        int determine_path_type(const GMDMPath &path);

        /* @brief Helper function. computes the state on the GMDM path at normalized "time" t, where t is between 0 and 1
         * Note: t=0 means return the start pose, t=1 returns the goal pose, and t=0.5 returns the midpoint state along the path
         * I expect that nomially, this function will only be called as a subroutine for the interpolate_path function with signature 
         * "std::vector<GMDMState> interpolate_path(const GMDMPath &path)"
         * If you really just one state along the path, and do not want to interpolate along the entire path, set "interpolated_once = false" and pass some dummy variables for p1 and p2.
         */
        GMDMState interpolate_path(const GMDMPath &path, double t, bool &interpolated_once, PoseSE2 &p1, PoseSE2 &p2);

        /* @brief helper function to interpolate along a motion primitive */
        PoseSE2 motion_primitive(const PoseSE2 &p, const GMDMSegment &seg, double t);

        /* @brief returns the set of states interlopated along a gmdm path specified by time stampes in vector t
         * each element of t must be between 0 and 1. The function does not check if the elements in t are in order.
         */
        std::vector<GMDMState> interpolate_path(const GMDMPath &path);

        /* @brief computes the interpolated states between the start and goal poses for the fastest GMDM path. */
        std::vector<GMDMState> interpolate_fastest_path() { return interpolate_path(get_fastest_path()); }

        /* @brief computes the interpolated states between the start and goal poses for the fastest GMDM path*/

        /* @brief this needs to be called any time one of the key parameters is changed.
         * ensures internal data structures are set up so that fast access to different path types, etc. is achieved.
         *
         */
        void reset();


        /* @brief pritns the GMDM problem definition (e.g., start and goal pose, defined parameters)*/
        void print_problem_definition();

        /* @brief prints the properties of the specified GMDM path */
        void print_gmdm_path(const GMDMPath &path);

        /* @brief prints properties for all GMDM paths. */
        void print_all_gmdm_paths();

        /**
         * @brief Writes a vector of GMDMState objects to a CSV file.
         * * @param filename The name of the file to create (e.g., "path_data.csv").
         * @param states The vector of GMDMState objects to write.
         * @return true if the file was written successfully, false otherwise.
         */
        bool write_gmdm_states_to_csv(const std::string &filename, const std::vector<GMDMState> &states);

    private:
        /* @brief a helpful constant. Turning rate omega = 0 rad/s is the same as going straight. NOTE: omega < 0 is a right turn, and omega > 0 is a left turn.*/
        const double OMEGA_STRAIGHT_ = 0.0;

        /* @brief a threshold for if a number is basically zero.
         * I apparently needed to use this in a very old version of my code for precision reasons, but I never documented why.
         */
        const double NEAR_ZERO_THRESHOLD = 1e-12;

        // Constants for different path types
        const int CSC_PATH_TYPE = 1;
        const int CCC_PATH_TYPE = 2;

        // key member variables defining scope of GMDM paths considered

        double v_min_;
        double v_max_;
        double omega_max_;
        double num_v_;
        double num_omega_;
        bool use_v_max_on_straight_;

        double time_step_{0.1}; // time step for interpolating path

        PoseSE2 pose_start_;
        PoseSE2 pose_goal_;

        /* True if the solver has been called for the given problem. Becomes false if a reset is required due to a change in parameters or if problem definition changed */
        bool is_solved_{false};

        /* @brief stores all linear speeds considered in order. v_set_[0] is v_min_ and v_set_[v_set_.size()-1] is v_max_ */
        std::vector<double> v_set_;

        /* @brief stores all turnig rates considered in order. omega_set_[0] is -omega_max_ and omega_set_[omega_set_.size()-1] is omega_max_*/
        std::vector<double> omega_set_;

        /* @brief cache for all possible path types to consider for the given key parameters. */
        std::vector<gmdm::GMDMPath> all_path_types_;

        /* @brief after calling solve(), this stores the rank of each GMDM path according to travel time.
         * e.g., if rank_[0] = 10, then GMDM path stored in all_path_types_[10] has the fastest travel time.
         * Note: rank_ always has the same size as all_path_types_
         */
        std::vector<size_t> rank_;

        /* @brief keeps track if the GMDM was reset/reinitialized after a key parameter or pose was changed or not. */
        bool is_reset_{false};

        // helper functions

        /* @brief constructs list of linear speeds considering min speed, max speed, and number of speeds */
        void construct_v_set();

        /* @brief constructs list of turning rates considered max turning rate and number of turning rates */
        void construct_omega_set();

        /* @brief determines all possible gmdm path types for given parameters and caches them for quick reference */
        void determine_all_path_types();

        /* @brief modulo operation, where mod(a,m) = a - m * floor (a/m). Follows convention where mod(a,0)=a*/
        double mod(double a, double m);

        /* @brief checks if a number is near zero. If so, set it to zero. Helps with robustness apparently */
        double near_zero(double num);

        /* @brief returns -1 if val < 0, +1 if val >0, and 0 if val = 0*/
        int sgn(double val);

        /* @brief a helper function to create a normalized time vector. used for interpolating states. */
        std::vector<double> create_time_vector(double travel_time);

    }; // class GMDM

} // namespace gmdm

#endif // GMDM_HPP