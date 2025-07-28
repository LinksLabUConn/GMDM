// Copyright (c) 2025 LINKS Lab UConn. All rights reserved.
// SPDX-License-Identifier: MIT
// Author: James P. Wilson (james.wilson@uconn.edu)

// This file provides example code on how to use the GMDM class.
// Specifically, it reproduces the results from the IEEE TRO article in Figure 6.

#include <iostream>
#include <iomanip> 
#include <vector>
#include <chrono>  

#include "gmdm/GMDM.hpp"

int main()
{

    // define GMDM parameters and initialize
    double v_min = 0.3;
    double v_max = 1.0;
    int num_v = 2;
    double omega_max = 1.0;
    int num_omega = 1;
    bool use_vmax_on_straight = true;

    // instantiate gmdm object
    gmdm::GMDM gmdm(v_min,v_max,omega_max,num_v,use_vmax_on_straight,num_omega);

    // define the start pose and goal poses
    gmdm::PoseSE2 start = {0.0, 0.0, 0.0};

    std::vector<gmdm::PoseSE2> goals;
    std::vector<double> xs = {-2,0,2};
    std::vector<double> ys = {-2,0,2};
    double theta = gmdm::PI * 2.0 / 3.0;
    for (auto x : xs)
    {
        for (auto y : ys)
        {
            if (x == 0 && y == 0)
                continue;
            goals.push_back({x,y,theta});
        }
    }

    // find the fastest path for each goal pose, interpolate states along the fastest path, and save to csv files.
    size_t counter = 0;
    gmdm.set_start_pose(start);
    for (auto& goal : goals)
    {
        gmdm.set_goal_pose(goal);
        auto start = std::chrono::high_resolution_clock::now();
        gmdm.solve_all();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = end - start;

        // Cast the duration to a double representing seconds
        auto duration_in_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration);

        
        //print problem
        gmdm.print_problem_definition();
        std::cout << "The best path for this problem is: \n";
        gmdm.print_gmdm_path(gmdm.get_fastest_path());

        // Set the output format to scientific and print
        std::cout << std::scientific;
        std::cout << "Function took " << duration_in_seconds.count() << " seconds." << std::endl;

        // construct path and print to disk
        std::vector<gmdm::GMDMState> states = gmdm.interpolate_fastest_path();
        gmdm.write_gmdm_states_to_csv("results/gmdm_path_states_" + std::to_string(counter++) + ".csv", states);
    }

    return 0;
}