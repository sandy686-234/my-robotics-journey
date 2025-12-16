(set-logic QF_LRA)
(set-option :produce-unsat-cores true)
(set-option :produce-models true)

; ----- (Optional) PDC windows: kept only as evidence; not used as constraints -----
; window 0: [0.012787916, 32.012787916] lhs=14.570164 rhs=32.000000
; (assert (! (<= 14.570164082 32.000000000) :named w0))
; window 1: [0.012787916, 33.012787916] lhs=29.154605 rhs=33.000000
; (assert (! (<= 29.154604961 33.000000000) :named w1))
; window 2: [0.012787916, 54.012787916] lhs=45.073729 rhs=54.000000
; (assert (! (<= 45.073728915 54.000000000) :named w2))
; window 3: [0.012787916, 66.012787916] lhs=61.245898 rhs=66.000000
; (assert (! (<= 61.245897855 66.000000000) :named w3))
; window 4: [0.012787916, 76.012787916] lhs=78.673205 rhs=76.000000
; (assert (! (<= 78.673204795 76.000000000) :named w4))
; window 5: [0.012787916, 88.012787916] lhs=92.351420 rhs=88.000000
; (assert (! (<= 92.351419685 88.000000000) :named w5))

; ----- Variables: start times of corridor-related legs -----
(declare-fun e_A_leg1 () Real)
(declare-fun e_A_leg2 () Real)
(declare-fun e_B_leg1 () Real)
(declare-fun e_B_leg2 () Real)

; ----- Per-leg release/deadline bounds with WCET + epsilon -----
(assert (<= 0.012788 e_A_leg1))
(assert (<= (+ e_A_leg1 15.919124 0.100000) 54.012788))

(assert (<= 0.012788 e_A_leg2))
(assert (<= (+ e_A_leg2 17.427307 0.100000) 76.012788))

(assert (<= 0.012788 e_B_leg1))
(assert (<= (+ e_B_leg1 16.172169 0.100000) 66.012788))

(assert (<= 0.012788 e_B_leg2))
(assert (<= (+ e_B_leg2 13.678215 0.100000) 88.012788))

; ===== Scheme A addition: per-agent chained precedence (serial execution) =====
; A: leg1 must finish before leg2 starts
(assert (<= (+ e_A_leg1 15.919124) e_A_leg2))
; B: leg1 must finish before leg2 starts
(assert (<= (+ e_B_leg1 16.172169) e_B_leg2))

; ----- Corridor mutual exclusion (A vs B pairs, with epsilon gap) -----
(assert (! (or (<= (+ e_A_leg1 15.919124 0.100000) e_B_leg1)
               (<= (+ e_B_leg1 16.172169 0.100000) e_A_leg1))
          :named mutex_A_leg1_B_leg1))
(define-fun A_leg1_before_B_leg1 () Bool
  (<= (+ e_A_leg1 15.919124 0.100000) e_B_leg1))
(define-fun B_leg1_before_A_leg1 () Bool
  (<= (+ e_B_leg1 16.172169 0.100000) e_A_leg1))

(assert (! (or (<= (+ e_A_leg1 15.919124 0.100000) e_B_leg2)
               (<= (+ e_B_leg2 13.678215 0.100000) e_A_leg1))
          :named mutex_A_leg1_B_leg2))
(define-fun A_leg1_before_B_leg2 () Bool
  (<= (+ e_A_leg1 15.919124 0.100000) e_B_leg2))
(define-fun B_leg2_before_A_leg1 () Bool
  (<= (+ e_B_leg2 13.678215 0.100000) e_A_leg1))

(assert (! (or (<= (+ e_A_leg2 17.427307 0.100000) e_B_leg1)
               (<= (+ e_B_leg1 16.172169 0.100000) e_A_leg2))
          :named mutex_A_leg2_B_leg1))
(define-fun A_leg2_before_B_leg1 () Bool
  (<= (+ e_A_leg2 17.427307 0.100000) e_B_leg1))
(define-fun B_leg1_before_A_leg2 () Bool
  (<= (+ e_B_leg1 16.172169 0.100000) e_A_leg2))

(assert (! (or (<= (+ e_A_leg2 17.427307 0.100000) e_B_leg2)
               (<= (+ e_B_leg2 13.678215 0.100000) e_A_leg2))
          :named mutex_A_leg2_B_leg2))
(define-fun A_leg2_before_B_leg2 () Bool
  (<= (+ e_A_leg2 17.427307 0.100000) e_B_leg2))
(define-fun B_leg2_before_A_leg2 () Bool
  (<= (+ e_B_leg2 13.678215 0.100000) e_A_leg2))

(check-sat)

; Return absolute start times (same origin as t0)
(get-value (e_A_leg1 e_A_leg2 e_B_leg1 e_B_leg2))

; Return who-first flags (for easy reading of corridor order)
(get-value (A_leg1_before_B_leg1 B_leg1_before_A_leg1
            A_leg1_before_B_leg2 B_leg2_before_A_leg1
            A_leg2_before_B_leg1 B_leg1_before_A_leg2
            A_leg2_before_B_leg2 B_leg2_before_A_leg2))
