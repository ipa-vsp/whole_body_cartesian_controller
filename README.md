# whole_body_cartesian_controller

```
[ ROS 2 Command Topic / Action ]
                │
                │ trajectory_msgs or geometry_msgs
                ▼
      ┌───────────────────┐
      │ rt_command_buffer │
      └─────────┬─────────┘
                │ X_des ∈ SE(3)
                │                                    [ Hardware State Interfaces ]
                │                                                  │
                │                                                  │ joint pos / base odom
                ▼                                                  ▼
   ┌──────────────────────────────────────────────────────────────────────────────┐
   │ WbCartesianController::update(time, period)         Line 185                 │
   │                                                                              │
   │   1. Read state interfaces ──▶ q ∈ ℝ^nq                                      │
   │   2. Read command buffer   ──▶ X_des ∈ SE(3)                                 │
   │                                                                              │
   │   3. computeTask(q, X_des)                          Line 252                 │
   │        Kinematics & placements   ──▶ oMf[ee_id] ∈ SE(3)                      │
   │        Relative pose on SE(3)    ──▶ iMd = (oMf)⁻¹ · X_des                   │
   │        Lie algebra error         ──▶ err = log₆(iMd) ∈ se(3) ≅ ℝ⁶            │
   │        Analytic task Jacobian    ──▶ J_task = -Jlog₆(iMd⁻¹) · J_frame        │
   │                                                                              │
   │   4. computeQP(q, dt)                               Line 302                 │
   │        Cost: H = Jᵀ W J + Λ + λ_lm I,   g = α Jᵀ W err                       │
   │        Constraints: A_eq · v = 0,       l_box ≤ v ≤ u_box                    │
   │        ProxQP Solve              ──▶ v = q̇ ∈ ℝ^nv                            │
   │        Manifold Retraction       ──▶ q_next = q ⊕ (v · dt)                   │
   │                                                                              │
   │   5. Write commands to hardware interfaces          Line 200                 │
   └──────────────────────────────────────┬───────────────────────────────────────┘
                                          │
                                          │ v (velocity) and/or q_next (position)
                                          ▼
                             [ Hardware Command Interfaces ]
```