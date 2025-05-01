// Copyright (C) 2024 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// private_vehicle_params.template.hpp
#pragma once

#include <vector>
#include <casadi/casadi.hpp>

// these are toy parameters and small sim2real gap will require tuning these parameters
const std::vector<casadi::DM> private_vehicle_parameters = {1.15000000e+01, 1.12815000e+02, 1.14800000e+02, 0.00000000e+00,
                                                            0.00000000e+00, 2.00000000e-02, 0.00000000e+00, 0.00000000e+00,
                                                            0.00000000e+00, 1.60000000e-01, 1.60000000e-01, 1.60000000e-01,
                                                            0.00000000e+00, -5.50000000e+00, -1.27000000e+01, -1.45700000e+01,
                                                            -1.20000000e-01, -1.20000000e-01, -1.20000000e-01, 0.00000000e+00,
                                                            0.00000000e+00, 0.00000000e+00, 0.00000000e+00, -4.03000000e+00,
                                                            -6.22000000e+00, -5.18000000e+00, -7.00000000e-02, -7.00000000e-02,
                                                            -7.00000000e-02, -1.81800000e+01, -2.16600000e+01, -3.69900000e+01,
                                                            -1.55000000e+00, -1.55000000e+00, -1.55000000e+00, 0.00000000e+00,
                                                            1.00421848e+00, 1.00000000e+00, 1.00000000e+00, 1.00000000e+00,
                                                            1.00000000e+00, 1.00000000e+00, 1.00000000e+00, 0.00000000e+00,
                                                            0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                                                            0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                                                            0.00000000e+00, 0.00000000e+00, 0.00000000e+00};
// ── Direction-dependent coefficients  (fwd0 … fwd3, rev0 … rev3) ────────────────
const std::vector<casadi::DM> sgn_qdot_k = {50.0, 50.0, 50.0, 50.0,  50.0, 50.0, 50.0, 50.0};
const std::vector<casadi::DM> viscous   = { 5,5,5,5,   5,5,5,5 };
const std::vector<casadi::DM> coulomb   = { 0,0,0,0,   0,0,0,0 };
const std::vector<casadi::DM> I_Grotor  = { 3,3,3,3,   3,3,3,3 };
const casadi::DM gravity = -9.81;
const std::vector<casadi::DM> joint_min = {-1000, -1000, -1000, -1000, -1000, -1000, 1, 0.01, 0.01, 0.01};
const std::vector<casadi::DM> joint_max = {1000, 1000, 1000, 1000, 1000, 1000, 5.50, 3.40, 3.40, 5.70};
const std::vector<casadi::DM> u_min = {-1, -1, -1, -5, -5, -1, -2.83664, -0.629139, -0.518764, -0.54};
const std::vector<casadi::DM> u_max = {1, 1, 2, 5, 5, 1, 2.83664, 0.629139, 0.518764, 0.54};

const std::vector<casadi::DM> Kp = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 2.0, 2.0, 2.0, 1.0};
const std::vector<casadi::DM> Ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.01, 0.01, 0.01};
const std::vector<casadi::DM> Kd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.1};
const std::vector<casadi::DM> EPS_TORQUE = {5e-1, 5e-1, 5e-1, 5e-1};