; SMT-based Multi-Robot Scheduler
(set-logic QF_LRA)
(set-option :produce-models true)
(set-option :produce-unsat-cores true)
(set-option :pp.decimal true)

; Decision variables: start times
(declare-const s_A_leg0 Real)
(declare-const s_A_leg1 Real)
(declare-const s_A_leg2 Real)
(declare-const s_B_leg0 Real)
(declare-const s_B_leg1 Real)
(declare-const s_B_leg2 Real)

; Makespan variable
(declare-const makespan Real)

; ===== Temporal Constraints =====
; Release time constraints
(assert (>= s_A_leg0 0.015100))
(assert (>= s_A_leg1 0.015100))
(assert (>= s_A_leg2 0.015100))
(assert (>= s_B_leg0 0.015100))
(assert (>= s_B_leg1 0.015100))
(assert (>= s_B_leg2 0.015100))

; Deadline constraints
(assert (<= (+ s_A_leg0 13.355984) 32.015100))
(assert (<= (+ s_A_leg1 14.592530) 54.015100))
(assert (<= (+ s_A_leg2 15.975031) 76.015100))
(assert (<= (+ s_B_leg0 13.369071) 33.015100))
(assert (<= (+ s_B_leg1 14.824488) 66.015100))
(assert (<= (+ s_B_leg2 12.538364) 88.015100))

; Precedence constraints (same robot)
(assert (>= s_A_leg1 (+ s_A_leg0 13.355984)))
(assert (>= s_A_leg2 (+ s_A_leg1 14.592530)))
(assert (>= s_B_leg1 (+ s_B_leg0 13.369071)))
(assert (>= s_B_leg2 (+ s_B_leg1 14.824488)))

; ===== Resource Mutual Exclusion =====
; Resource: corridor_1
(assert (! (or (<= (+ s_A_leg1 14.592530 0.100000) s_B_leg1) (<= (+ s_B_leg1 14.824488 0.100000) s_A_leg1)) :named mutex_corridor_1_A_leg1_B_leg1))
(assert (! (or (<= (+ s_A_leg1 14.592530 0.100000) s_B_leg2) (<= (+ s_B_leg2 12.538364 0.100000) s_A_leg1)) :named mutex_corridor_1_A_leg1_B_leg2))
(assert (! (or (<= (+ s_A_leg2 15.975031 0.100000) s_B_leg1) (<= (+ s_B_leg1 14.824488 0.100000) s_A_leg2)) :named mutex_corridor_1_A_leg2_B_leg1))
(assert (! (or (<= (+ s_A_leg2 15.975031 0.100000) s_B_leg2) (<= (+ s_B_leg2 12.538364 0.100000) s_A_leg2)) :named mutex_corridor_1_A_leg2_B_leg2))

; ===== Makespan =====
(assert (>= makespan (+ s_A_leg2 15.975031)))
(assert (>= makespan (+ s_B_leg2 12.538364)))

; ===== Objective: Minimize Makespan =====
(minimize makespan)

(check-sat)
; Get all start times
(get-value (makespan s_A_leg0 s_A_leg1 s_A_leg2 s_B_leg0 s_B_leg1 s_B_leg2))