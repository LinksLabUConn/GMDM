// Copyright (c) 2025 LINKS Lab UConn. All rights reserved.
// SPDX-License-Identifier: MIT
// Author: James P. Wilson (james.wilson@uconn.edu)

#include "gmdm/GMDM.hpp"


#include <cmath>
#include <numeric>  
#include <algorithm> 
#include <fstream> 
#include <string>
#include <iomanip> 

gmdm::GMDM::GMDM(double v_min, double v_max, double omega_max, int num_v, bool use_v_max_on_straight, int num_omega)
{
    set_v_min(v_min);
    set_v_max(v_max);
    set_omega_max(omega_max);
    set_num_v(num_v);
    set_num_omega(num_omega);
    set_use_v_max_on_straight(use_v_max_on_straight);

    reset();
}


void gmdm::GMDM::set_v_min(double v_min)
{
    assert(v_min > 0);
    v_min_ = v_min;
    is_reset_ = false;
}

void gmdm::GMDM::set_v_max(double v_max)
{
    assert(v_max > v_min_);
    v_max_ = v_max;
    is_reset_ = false;
}

void gmdm::GMDM::set_omega_max(double omega_max)
{
    assert(omega_max > 0);
    omega_max_ = omega_max;
    is_reset_ = false;
}

void gmdm::GMDM::set_num_v(int num_v)
{
    assert(num_v >= 1);
    num_v_ = num_v;
    is_reset_ = false;
}

void gmdm::GMDM::set_use_v_max_on_straight(bool use_v_max_on_straight)
{
    use_v_max_on_straight_ = use_v_max_on_straight;
    is_reset_ = false;
}

void gmdm::GMDM::set_num_omega(int num_omega)
{
    assert(num_omega >= 1);
    num_omega_ = num_omega;
    is_reset_ = false;
}

void gmdm::GMDM::set_start_pose(const PoseSE2& start)
{
    pose_start_ = start;
    is_solved_ = false;
    // do not necessarily need to reset; path types are the same if nothing else is changed!
}

void gmdm::GMDM::set_goal_pose(const PoseSE2& goal)
{
    pose_goal_ = goal;
    is_solved_ = false;
}

void gmdm::GMDM::set_time_step(double time_step)
{
    assert(time_step > 0 && time_step < std::numeric_limits<double>::infinity());
    time_step_ = time_step;
}

void gmdm::GMDM::solve_all()
{
    assert(is_reset_ == true);
    
    // calculate the durations of each segment for each path
    for (auto& path : all_path_types_)
        solve_one(path);


    // fills rank_ with {0, 1, 2, ...}
    std::iota(rank_.begin(), rank_.end(), 0);

    // rank the paths
    std::sort(rank_.begin(), rank_.end(),
        [&](int i, int j){
            return all_path_types_[i].travel_time() < all_path_types_[j].travel_time();
        });

    is_solved_ = true;
}

void gmdm::GMDM::solve_one(GMDMPath& path)
{
    int path_type = determine_path_type(path);
    assert(path_type != OTHER_PATH_TYPE);

    // DO MATH
    // for details, see section III of the GMDM paper in IEEE TRO

    //precompute some common stuff
    double r1 = path.seg1.v / path.seg1.omega;
    double r3 = path.seg3.v / path.seg3.omega;

    double a = pose_goal_.x-pose_start_.x + r1*std::sin(pose_start_.theta) - r3*std::sin(pose_goal_.theta);
    double b = pose_goal_.y-pose_start_.y - r1*std::cos(pose_start_.theta) + r3*std::cos(pose_goal_.theta);

    double invsqrtab = 1.0 / std::sqrt(a*a + b*b);
    
    if (path_type == CCC_PATH_TYPE)
    {
        double r2 = path.seg2.v / path.seg2.omega;
        double r12 = r1 - r2;
        double r23 = r2 - r3;
        double q1 = (a*a + b*b + r12*r12 - r23*r23) / (2*r12) * invsqrtab;
        double q2 = (a*a + b*b + r23*r23 - r12*r12) / (2*r23) * invsqrtab;

        // check if path can connect start and goal pose
        if (std::abs(q1) <= 1 && std::abs(q2) <= 1)
        {
            // a feasible path exists!

            //precompute more stuff
            double theta1 = PI - std::asin(q1) - std::atan2(-b,a);
            double theta2 = PI - std::asin(q2) - std::atan2(-b,a);

            double theta10 = mod(near_zero(theta1-pose_start_.theta), 2*PI*sgn(path.seg1.omega));
            double theta21 = mod(near_zero(theta2-theta1), 2*PI*sgn(path.seg2.omega));
            double theta32 = mod(near_zero(pose_goal_.theta-theta2),2*PI*sgn(path.seg3.omega));

            // finally, compute travel time durations of each segment.
            path.seg1.tau = theta10 / path.seg1.omega;
            path.seg2.tau = theta21 / path.seg2.omega;
            path.seg3.tau = theta32 / path.seg3.omega;
        }
        else
        {   //path is infeasible, so set durations to infinity
            path.seg1.tau = std::numeric_limits<double>::infinity();
            path.seg2.tau = std::numeric_limits<double>::infinity();
            path.seg3.tau = std::numeric_limits<double>::infinity();
        }
    }
    else if (path_type == CSC_PATH_TYPE)
    {
        double r31 = r3 - r1;
        double q = -r31 * invsqrtab;

        // check if path can connect start and goal pose
        if (std::abs(q) <= 1)
        {
            // a feasible path exists!

            //precompute more stuff
            double theta1 = std::asin(q) - std::atan2(-b,a);
            double theta10 = mod(near_zero(theta1-pose_start_.theta), 2*PI*sgn(path.seg1.omega));

            double delta2 = std::sqrt(a*a + b*b - r31*r31);

            double theta31 = mod(near_zero(pose_goal_.theta-theta1),2*PI*sgn(path.seg3.omega));


            // finally, compute travel time durations of each segment.
            path.seg1.tau = theta10 / path.seg1.omega;
            path.seg2.tau = delta2 / path.seg2.v;
            path.seg3.tau = theta31 / path.seg3.omega;
        }
        else
        {   //path is infeasible, so set durations to infinity
            path.seg1.tau = std::numeric_limits<double>::infinity();
            path.seg2.tau = std::numeric_limits<double>::infinity();
            path.seg3.tau = std::numeric_limits<double>::infinity();
        }
    }
}

std::vector<double> gmdm::GMDM::create_time_vector(double travel_time)
{
    std::vector<double> time_vec;

    // Handle edge cases to avoid division by zero or infinite loops
    if (travel_time <= NEAR_ZERO_THRESHOLD || time_step_ <= NEAR_ZERO_THRESHOLD) {
        if (travel_time > NEAR_ZERO_THRESHOLD) {
            time_vec.push_back(0.0);
            time_vec.push_back(1.0);
        }
        return time_vec;
    }

    // To be efficient, reserve memory upfront to prevent reallocations.
    // The number of elements will be roughly travel_time / time_step_ plus a couple more.
    time_vec.reserve(static_cast<size_t>(travel_time / time_step_) + 2);

    // Generate the sequence of time stamps from 0.0 up to (but not including) 1.0
    for (double t = 0.0; t < travel_time; t += time_step_) {
        time_vec.push_back(t / travel_time);
    }

    // Always add the final ratio, which is 1.0. This ensures the endpoint is included
    // This implementation assumes you don't want a duplicate 1.0 if travel_time
    // is a perfect multiple of time_step_.
    time_vec.push_back(1.0);

    return time_vec;
}


gmdm::PoseSE2 gmdm::GMDM::motion_primitive(const PoseSE2& p, const GMDMSegment& seg, double t)
{
    if (t < 0)
        t = 0;
    else if (t > 1)
        t = 1;

    PoseSE2 pose;
    double travel_time = t*seg.tau; // the relative time stamp of the pose we are calculating
    if (seg.omega == OMEGA_STRAIGHT_) // straight line segment
    {
        // these are Equations 6a-6c in the GMDM paper
        pose.x = p.x + seg.v*travel_time*std::cos(p.theta);
        pose.y = p.y + seg.v*travel_time*std::sin(p.theta);
        pose.theta = p.theta;
    }
    else // turning segment
    {
        // these are Equations 5a-5c in the GMDM paper
        pose.x = p.x - seg.v*(std::sin(p.theta)-std::sin(p.theta+seg.omega*travel_time))/seg.omega;
        pose.y = p.y + seg.v*(std::cos(p.theta)-std::cos(p.theta+seg.omega*travel_time))/seg.omega;
        pose.theta = mod(p.theta+seg.omega*travel_time, 2*PI);
    }
    return pose;
}

gmdm::GMDMState gmdm::GMDM::interpolate_path(const GMDMPath& path, double t, bool& interpolated_once, PoseSE2& p1, PoseSE2& p2)
{
    assert(is_solved_ == true);
    assert(path.travel_time() >= 0 && path.travel_time() < std::numeric_limits<double>::infinity());

    double travel_time = path.travel_time();
    GMDMState state;
    state.t = travel_time*t;

    // I am assuming it is unlikely only ever one state will be interpolated on a path
    // and this function will be called in a loop
    // hence, will compute transition poses if not called yet
    if (interpolated_once == false)
    {
        p1 = motion_primitive(pose_start_, path.seg1, 1);
        p2 = motion_primitive(p1, path.seg2, 1);
        
        interpolated_once = true;
    }

    // determine which segment this point will fall on
    if (state.t <= path.seg1.tau)
    {
        // on segment 1
        state.v = path.seg1.v;
        state.omega = path.seg1.omega;

        double t1;
        if (path.seg1.tau == 0)
            t1 = 0;
        else
            t1 = state.t / path.seg1.tau;
        state.pose = motion_primitive(pose_start_, path.seg1, t1);

    }
    else if (state.t <= path.seg1.tau + path.seg2.tau)
    {
        // on segment 2
        state.v = path.seg2.v;
        state.omega = path.seg2.omega;

        double t2 = (state.t - path.seg1.tau)/path.seg2.tau;
        state.pose = motion_primitive(p1, path.seg2, t2);
    }
    else 
    {
        // on segment 3
        state.v = path.seg3.v;
        state.omega = path.seg3.omega;

        double t3 = (state.t - path.seg1.tau - path.seg2.tau) / path.seg3.tau;
        state.pose = motion_primitive(p2, path.seg3, t3);
    }

    return state;    
}



std::vector<gmdm::GMDMState> gmdm::GMDM::interpolate_path(const GMDMPath& path)
{
    assert(is_solved_ == true);
    std::vector<gmdm::GMDMState> interpolated_path;

    // initialize the normalized time vector
    std::vector<double> t = create_time_vector(path.travel_time());


    
    interpolated_path.reserve(t.size());

    // p1 and p2 are the transition points between seg1-seg2 and seg2-seg3, respectively.
    PoseSE2 p1;
    PoseSE2 p2;
    bool interpolated_once = false;

    for (int i = 0; i < t.size(); i++)
        interpolated_path.push_back(interpolate_path(path,t[i], interpolated_once, p1, p2));

    return interpolated_path;
}

int gmdm::GMDM::determine_path_type(const GMDMPath& path)
{
    
    // first establish that both the first and third segments are curves
    if (path.seg1.omega == OMEGA_STRAIGHT_ || path.seg3.omega == OMEGA_STRAIGHT_)
        return OTHER_PATH_TYPE;
    // checks if it's a valid CSC path
    else if (path.seg2.omega == OMEGA_STRAIGHT_)
        return CSC_PATH_TYPE;
    //check if it's a valid CCC path
    else if (std::signbit(path.seg1.omega) != std::signbit(path.seg2.omega) && std::signbit(path.seg2.omega) != std::signbit(path.seg3.omega))
        return CCC_PATH_TYPE;
    else
        return OTHER_PATH_TYPE;

}

void gmdm::GMDM::construct_v_set()
{
    v_set_.clear();
    v_set_.shrink_to_fit();
    v_set_.reserve(num_v_);
    if (num_v_ == 1)
    {
        v_set_.push_back(v_max_);
    }
    else
    {
        double step = (v_max_- v_min_) / (num_v_ - 1);

        for (int i = 0; i < num_v_; i++)
        {
            v_set_.push_back(v_min_ + i * step);
        }
        v_set_.back() = v_max_;
    }
}

void gmdm::GMDM::construct_omega_set()
{
    omega_set_.clear();
    omega_set_.shrink_to_fit();
    omega_set_.reserve(num_omega_ * 2 + 1);
    double omega = -1.0*omega_max_;
    double step = omega_max_ / num_omega_;
    for (int i = 0; i < num_omega_; i++)
    {
        omega_set_.push_back(omega);
        omega += step;
    }
    omega_set_.push_back(0);
    omega = step;
    for (int i = 0; i < num_omega_; i++)
    {
        omega_set_.push_back(omega);
        omega += step;
    }
    omega_set_.back() = omega_max_;
}


void gmdm::GMDM::determine_all_path_types()
{
    int num_path_types;
    if (use_v_max_on_straight_)
        num_path_types = 4*num_omega_*num_omega_*num_v_*num_v_+2*num_omega_*num_omega_*num_omega_*num_v_*num_v_*num_v_;
    else
        num_path_types = 4*num_omega_*num_omega_*num_v_*num_v_*num_v_+2*num_omega_*num_omega_*num_omega_*num_v_*num_v_*num_v_;
    all_path_types_.clear();
    all_path_types_.shrink_to_fit();
    all_path_types_.reserve(num_path_types);

    // now populate all_path_types
    
    // first do CSC path types
    for (const auto& o1 : omega_set_)
    {
        if (o1 == OMEGA_STRAIGHT_)
            continue;
        for (const auto& o3 : omega_set_)
        {
            if (o3 == OMEGA_STRAIGHT_)
                continue;
            
            for (const auto& v1 : v_set_)
            {
                for (const auto& v2 : v_set_)
                {
                    if (use_v_max_on_straight_ && v2 != v_max_)
                        continue;

                    for (const auto& v3 : v_set_)
                    {
                        all_path_types_.push_back({{v1,o1},{v2,0},{v3,o3}});
                    }
                }
            }

        }
    }

    // now do CCC path types
    for (const auto& o1 : omega_set_)
    {
        if (o1 == OMEGA_STRAIGHT_)
            continue;
        for (const auto& o2 : omega_set_)
        {
            if (o2 == OMEGA_STRAIGHT_ || std::signbit(o1) == std::signbit(o2))
                continue;

            for (const auto& o3 : omega_set_)
            {
                if (o3 == OMEGA_STRAIGHT_ || std::signbit(o2) == std::signbit(o3))
                    continue;
                
                for (const auto& v1 : v_set_)
                {
                    for (const auto& v2 : v_set_)
                    {
                        for (const auto& v3 : v_set_)
                        {
                            all_path_types_.push_back({{v1,o1},{v2,o2},{v3,o3}});
                        }
                    }
                }
            }
        }
    }

    // the number of path types added should match the number of calculated path types
    assert(num_path_types == all_path_types_.size());

    // initialize
    rank_.resize(all_path_types_.size());
}

double gmdm::GMDM::mod(double a, double m)
{
    if (m == 0.)
        return a;
    return a - m * std::floor(a / m);    
}

double gmdm::GMDM::near_zero(double num)
{
    if (std::abs(num) < NEAR_ZERO_THRESHOLD)
        return 0;
    else
        return num;
}

int gmdm::GMDM::sgn(double val)
{
    if (val < 0)
        return -1;
    if (val > 0)
        return 1;
    return 0;
}

void gmdm::GMDM::reset()
{
    construct_v_set();
    construct_omega_set();
    determine_all_path_types();

    is_solved_ = false;
    is_reset_ = true;
}


void gmdm::GMDM::print_problem_definition()
{
    std::cout << "\n////////////////////////////////////////////\n";
    std::cout << "GMDM Problem Definition\n";
    std::cout << "Start Pose (x,y,theta) = (" << pose_start_.x << "," << pose_start_.y << "," << pose_start_.theta <<")\n";
    std::cout << "Goal Pose (x,y,theta) = (" << pose_goal_.x << "," << pose_goal_.y << "," << pose_goal_.theta << ")\n";
    std::cout << "Speeds V = {";
    for (int i = 0; i < v_set_.size(); i++)
    {
        std::cout << v_set_[i];
        if (i != v_set_.size()-1)
            std::cout << ",";
    }
    std::cout << "}\n";
    std::cout << "Turning Rates Omega = {";
    for (int i = 0; i < omega_set_.size(); i++)
    {
        std::cout << omega_set_[i];
        if (i != omega_set_.size()-1)
            std::cout << ",";
    }
    std::cout << "}\n";
    std::cout << "Use max{V} on straight = " << use_v_max_on_straight_ << std::endl;
    std::cout << "Number of GMDM Paths = " << all_path_types_.size() << std::endl;
    std::cout << "////////////////////////////////////////////\n";
}

void gmdm::GMDM::print_gmdm_path(const GMDMPath& path)
{
    std::vector<GMDMSegment> segs;
    segs.push_back(path.seg1);
    segs.push_back(path.seg2);
    segs.push_back(path.seg3);
    std::cout << "{(v_1,omega_1,tau1),(v_2,omega_2,tau2),(v_3,omega_3,tau3)} = {";
    for (int i = 0; i < segs.size(); i++)
    {
        std::cout << "(" << segs[i].v << "," << segs[i].omega << "," << segs[i].tau << ")";
        if (i != segs.size()-1)
            std::cout << ",";
    }
    std::cout << "}" << " | T = " << path.travel_time() << " s" << "\n";
}

void gmdm::GMDM::print_all_gmdm_paths()
{   std::cout << "\n////////////////////////////////////////////\n";
    std::cout << "Printing properties of all GMDM Paths\n";
    if (is_solved_)
    {
        for (int i = 0; i < rank_.size(); i++)
            print_gmdm_path(all_path_types_[rank_[i]]);
    }
    else
    {
        for (auto& path : all_path_types_)
            print_gmdm_path(path);
    }
    std::cout << "////////////////////////////////////////////\n";
}

bool gmdm::GMDM::write_gmdm_states_to_csv(const std::string& filename, const std::vector<GMDMState>& states)
{
    // 1. Create an output file stream object
    std::ofstream file_out(filename);

    // 2. Check if the file was opened successfully
    if (!file_out.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return false;
    }

    // Set precision for floating-point numbers to ensure consistency
    file_out << std::fixed << std::setprecision(6);

    // 3. Write the header row
    file_out << "t,x,y,theta,v,omega\n";

    // 4. Loop through each state and write its data to the file
    for (const auto& state : states) {
        file_out << state.t << ","
                 << state.pose.x << ","
                 << state.pose.y << ","
                 << state.pose.theta << ","
                 << state.v << ","
                 << state.omega << "\n";
    }

    // 5. The file stream will be automatically closed when 'file_out' goes out of scope.
    std::cout << "Successfully wrote data to " << filename << std::endl;
    return true;
}
