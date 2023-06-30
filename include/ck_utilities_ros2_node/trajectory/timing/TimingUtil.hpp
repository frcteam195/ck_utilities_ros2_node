#pragma once

#include "ck_utilities_ros2_node/CKMath.hpp"
#include "ck_utilities_ros2_node/team254_geometry/Geometry.hpp"
#include "ck_utilities_ros2_node/trajectory/timing/MinMaxAcceleration.hpp"
#include "ck_utilities_ros2_node/trajectory/Trajectory.hpp"
#include "ck_utilities_ros2_node/trajectory/DistanceView.hpp"
#include "TimingConstraint.hpp"
#include "TimedState.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <type_traits>
#include <vector>

namespace ck
{
    namespace trajectory
    {
        namespace timing
        {
            template <class S, class T>
            struct ConstrainedState
            {
                S state;
                T heading;
                double distance;

                double max_translational_velocity;
                T max_angular_velocity;
                double min_translational_acceleration;
                T min_angular_acceleration;
                double max_acceleration;
                T max_angular_acceleration;
            };

            class TimingUtil
            {
                // static_assert(std::is_base_of<ck::team254_geometry::State<S>, S>::value, "S must inherit from State<S>");
            public:
                template <class S, class T>
                static Trajectory<TimedState<S>, TimedState<T>> timeParameterizeTrajectory(
                    bool reverse,
                    ck::trajectory::DistanceView<S, T> &distance_view,
                    double step_size,
                    std::vector<TimingConstraint<S> *> &constraints,
                    double start_velocity,
                    double end_velocity,
                    double max_translational_velocity,
                    double max_abs_acceleration,
                    double max_abs_deceleration,
                    bool apply_smoothing)
                {
                    int num_states = (int)std::ceil(distance_view.last_interpolant() / step_size + 1);
                    std::vector<S> states;
                    std::vector<T> headings;
                    for (int i = 0; i < num_states; ++i)
                    {
                        states.push_back(distance_view.sample(math::min(i * step_size, distance_view.last_interpolant())).state_);
                        headings.push_back(distance_view.sample(math::min(i * step_size, distance_view.last_interpolant())).heading_);
                    }
                    return timeParameterizeTrajectory(reverse, states, headings, constraints, start_velocity, end_velocity, max_translational_velocity, max_abs_acceleration, max_abs_deceleration, apply_smoothing);
                }

                template <class S, class T>
                static ck::trajectory::Trajectory<TimedState<S>, TimedState<T>> timeParameterizeTrajectory(
                    bool reverse,
                    std::vector<S> &states,
                    std::vector<T> &headings,
                    std::vector<TimingConstraint<S> *> &constraints,
                    double start_velocity,
                    double end_velocity,
                    double max_translational_velocity,
                    double max_abs_acceleration,
                    double max_abs_deceleration,
                    bool apply_smoothing)
                {
                    std::vector<ConstrainedState<S, T>> constraint_states;
                    constraint_states.reserve(states.size());
                    constexpr double kEpsilon = 1e-6;

                    // double non_lin_cutoff = 10.0;
                    double min_accel_pct = 0.5;
                    double accel_power = 1;
                    double min_decel_pct = 0.5;
                    double decel_power = 2;

                    size_t non_lin_index = ceil(states.size() * 0.05);

                    // Forward pass. We look at pairs of consecutive states, where the start state has already been velocity
                    // parameterized (though we may adjust the velocity downwards during the backwards pass). We wish to find an
                    // acceleration that is admissible at both the start and end state, as well as an admissible end velocity. If
                    // there is no admissible end velocity or acceleration, we set the end velocity to the state's maximum allowed
                    // velocity and will repair the acceleration during the backward pass (by slowing down the predecessor).
                    ConstrainedState<S, T> predecessor;
                    predecessor.state = states[0];
                    predecessor.distance = 0.0;
                    predecessor.max_translational_velocity = start_velocity;
                    predecessor.min_translational_acceleration = -max_abs_acceleration;
                    predecessor.max_acceleration = max_abs_acceleration;
                    for (size_t i = 0; i < states.size(); ++i)
                    {
                        // Add the new state.
                        ConstrainedState<S, T> constraint_state;
                        constraint_state.state = states[i];
                        constraint_state.heading = headings[i];
                        double ds = constraint_state.state.distance(predecessor.state);
                        constraint_state.distance = ds + predecessor.distance;

                        // We may need to iterate to find the maximum end velocity and common acceleration, since acceleration
                        // limits may be a function of velocity.
                        while (true)
                        {
                            // Enforce global max velocity and max reachable velocity by global acceleration limit.
                            // vf = sqrt(vi^2 + 2*a*d)
                            constraint_state.max_translational_velocity = ck::math::min(max_translational_velocity, std::sqrt(predecessor.max_translational_velocity * predecessor.max_translational_velocity + 2.0 * predecessor.max_acceleration * ds));
                            if (std::isnan(constraint_state.max_translational_velocity))
                            {
                                throw;
                            }
                            // Enforce global max absolute acceleration.
                            constraint_state.min_translational_acceleration = -max_abs_acceleration;
                            constraint_state.max_acceleration = max_abs_acceleration;

                            // double dist_from_start = std::abs(states[i].getTranslation().norm() - states[0].getTranslation().norm());
                            // if (apply_smoothing && dist_from_start <= non_lin_cutoff)
                            if (apply_smoothing && i < non_lin_index)
                            {
                                // double pct = dist_from_start / non_lin_cutoff;
                                double pct = i / non_lin_index;
                                // double pct_norm = std::pow(ck::math::map(pct, 0.0, 1.0, min_accel_pct, 1.0), accel_power);
                                double pct_norm = std::pow(ck::math::map2(pct, min_accel_pct), accel_power);
                                constraint_state.max_acceleration *= pct_norm;
                                constraint_state.min_translational_acceleration *= pct_norm;
                            }

                            // At this point, the state is full constructed, but no constraints have been applied aside from
                            // predecessor
                            // state max accel.

                            // Enforce all velocity constraints.
                            for (const TimingConstraint<S> *constraint : constraints)
                            {
                                constraint_state.max_translational_velocity = ck::math::min(constraint_state.max_translational_velocity, constraint->getMaxVelocity(constraint_state.state));
                            }
                            if (constraint_state.max_translational_velocity < 0.0)
                            {
                                // This should never happen if constraints are well-behaved.
                                throw;
                            }

                            // Now enforce all acceleration constraints.
                            for (const TimingConstraint<S> *constraint : constraints)
                            {
                                MinMaxAcceleration min_max_accel = constraint->getMinMaxAcceleration(constraint_state.state, (reverse ? -1.0 : 1.0) * constraint_state.max_translational_velocity);
                                if (!min_max_accel.valid())
                                {
                                    // This should never happen if constraints are well-behaved.
                                    throw;
                                }
                                constraint_state.min_translational_acceleration = ck::math::max(constraint_state.min_translational_acceleration, reverse ? -min_max_accel.max_acceleration() : min_max_accel.min_acceleration());
                                constraint_state.max_acceleration = ck::math::min(constraint_state.max_acceleration, reverse ? -min_max_accel.min_acceleration() : min_max_accel.max_acceleration());
                            }
                            if (constraint_state.min_translational_acceleration > constraint_state.max_acceleration)
                            {
                                // This should never happen if constraints are well-behaved.
                                throw;
                            }

                            if (ds < kEpsilon)
                            {
                                break;
                            }
                            // If the max acceleration for this constraint state is more conservative than what we had applied, we
                            // need to reduce the max accel at the predecessor state and try again.
                            // TODO: Simply using the new max acceleration is guaranteed to be valid, but may be too conservative.
                            // Doing a search would be better.
                            double actual_acceleration = (constraint_state.max_translational_velocity * constraint_state.max_translational_velocity - predecessor.max_translational_velocity * predecessor.max_translational_velocity) / (2.0 * ds);
                            if (constraint_state.max_acceleration < actual_acceleration - kEpsilon)
                            {
                                predecessor.max_acceleration = constraint_state.max_acceleration;
                            }
                            else
                            {
                                if (actual_acceleration > predecessor.min_translational_acceleration + kEpsilon)
                                {
                                    predecessor.max_acceleration = actual_acceleration;
                                }
                                // If actual acceleration is less than predecessor min accel, we will repair during the backward
                                // pass.
                                break;
                            }
                            // ConsoleReporter.report("(intermediate) i: " + i + ", " + constraint_state.toString());
                        }
                        // ConsoleReporter.report("i: " + i + ", " + constraint_state.toString());
                        predecessor = constraint_state;
                        constraint_states.push_back(constraint_state);
                    }

                    for (size_t i = 0; i < constraint_states.size(); i++)
                    {
                        constraint_states[i].max_acceleration = max_abs_deceleration;
                        constraint_states[i].min_translational_acceleration = -max_abs_deceleration;
                    }

                    // for (ConstrainedState<S, T> cs : constraint_states)
                    // {
                    //     std::cout << cs.max_acceleration << std::endl;
                    //     std::cout << cs.min_translational_acceleration << std::endl << std::endl;
                    // }

                    // Backward pass.
                    ConstrainedState<S, T> successor;
                    successor.state = states[states.size() - 1];
                    successor.heading = headings[headings.size() - 1];
                    successor.distance = constraint_states[states.size() - 1].distance;
                    successor.max_translational_velocity = end_velocity;
                    successor.min_translational_acceleration = -max_abs_deceleration;
                    successor.max_acceleration = max_abs_deceleration;
                    for (int i = states.size() - 1; i >= 0; --i)
                    {
                        ConstrainedState<S, T> &constraint_state = constraint_states[i];
                        double ds = constraint_state.distance - successor.distance; // will be negative.

                        constraint_state.max_acceleration = max_abs_deceleration;
                        constraint_state.min_translational_acceleration = -max_abs_deceleration;

                        // double dist_to_end = std::abs(states[states.size()-1].getTranslation().norm() - states[i].getTranslation().norm());
                        // if (apply_smoothing && dist_to_end <= non_lin_cutoff)
                        if (apply_smoothing && i > ((int)states.size() - (int)non_lin_index))
                        {
                            // double pct = dist_to_end / non_lin_cutoff;
                            double pct = ((int)states.size() - i) / non_lin_index;
                            // double pct_norm = std::pow(ck::math::map(pct, 0.0, 1.0, min_decel_pct, 1.0), decel_power);
                            double pct_norm = std::pow(ck::math::map2(pct, min_decel_pct), decel_power);
                            constraint_state.max_acceleration *= pct_norm;
                            constraint_state.min_translational_acceleration *= pct_norm;
                        }

                        while (true)
                        {
                            // Enforce reverse max reachable velocity limit.
                            // vf = sqrt(vi^2 + 2*a*d), where vi = successor.
                            double new_max_translational_velocity = std::sqrt(successor.max_translational_velocity * successor.max_translational_velocity + 2.0 * successor.min_translational_acceleration * ds);
                            if (new_max_translational_velocity >= constraint_state.max_translational_velocity)
                            {
                                // No new limits to impose.
                                break;
                            }
                            constraint_state.max_translational_velocity = new_max_translational_velocity;
                            if (std::isnan(constraint_state.max_translational_velocity))
                            {
                                throw;
                            }

                            // Now check all acceleration constraints with the lower max velocity.
                            for (const TimingConstraint<S> *constraint : constraints)
                            {
                                MinMaxAcceleration min_max_accel = constraint->getMinMaxAcceleration(constraint_state.state, (reverse ? -1.0 : 1.0) * constraint_state.max_translational_velocity);
                                if (!min_max_accel.valid())
                                {
                                    throw;
                                }
                                constraint_state.min_translational_acceleration = ck::math::max(constraint_state.min_translational_acceleration, reverse ? -min_max_accel.max_acceleration() : min_max_accel.min_acceleration());
                                constraint_state.max_acceleration = ck::math::min(constraint_state.max_acceleration, reverse ? -min_max_accel.min_acceleration() : min_max_accel.max_acceleration());
                            }
                            if (constraint_state.min_translational_acceleration > constraint_state.max_acceleration)
                            {
                                throw;
                            }

                            if (ds > kEpsilon)
                            {
                                break;
                            }
                            // If the min acceleration for this constraint state is more conservative than what we have applied, we
                            // need to reduce the min accel and try again.
                            // TODO: Simply using the new min acceleration is guaranteed to be valid, but may be too conservative.
                            // Doing a search would be better.
                            double actual_acceleration = (constraint_state.max_translational_velocity * constraint_state.max_translational_velocity - successor.max_translational_velocity * successor.max_translational_velocity) / (2.0 * ds);
                            if (constraint_state.min_translational_acceleration > actual_acceleration + kEpsilon)
                            {
                                successor.min_translational_acceleration = constraint_state.min_translational_acceleration;
                            }
                            else
                            {
                                successor.min_translational_acceleration = actual_acceleration;
                                break;
                            }
                        }
                        successor = constraint_state;
                    }

                    // for (ConstrainedState<S, T> cs : constraint_states)
                    // {
                    //     std::cout << cs.max_translational_velocity << std::endl;
                    // }


                    // Integrate the constrained states forward in time to obtain the TimedStates.
                    std::vector<TimedState<S>> timed_states;
                    std::vector<TimedState<T>> timed_headings;
                    timed_states.reserve(states.size());
                    timed_headings.reserve(headings.size());
                    double t = 0.0;
                    double s = 0.0;
                    double v = 0.0;
                    for (size_t i = 0; i < states.size(); ++i)
                    {
                        ConstrainedState<S, T> &constrained_state = constraint_states[i];
                        // Advance t.
                        double ds = constrained_state.distance - s;
                        double accel = (constrained_state.max_translational_velocity * constrained_state.max_translational_velocity - v * v) / (2.0 * ds);
                        double heading_accel = 60.0; // why poofs why
                        double dt = 0.0;
                        if (i > 0)
                        {
                            timed_states[i - 1].set_acceleration(reverse ? -accel : accel);
                            if (std::abs(accel) > kEpsilon)
                            {
                                dt = (constrained_state.max_translational_velocity - v) / accel;
                            }
                            else if (std::abs(v) > kEpsilon)
                            {
                                dt = ds / v;
                            }
                            else
                            {
                                throw std::runtime_error("Value for accel and velocity is zero in TimingUtil");
                            }
                        }
                        t += dt;
                        if (std::isnan(t) || std::isinf(t))
                        {
                            throw std::runtime_error("Value for time is infinite in TimingUtil");
                        }

                        v = constrained_state.max_translational_velocity;
                        s = constrained_state.distance;
                        timed_states.push_back(TimedState<S>(constrained_state.state, t, reverse ? -v : v, reverse ? -accel : accel));
                        timed_headings.push_back(TimedState<T>(constrained_state.heading, t, reverse ? -v : v, reverse ? -heading_accel : heading_accel));
                    }
                    return Trajectory<TimedState<S>, TimedState<T>>(timed_states, timed_headings);
                }
            };
        } // namespace timing
    }     // namespace trajectory
} // namespace ck