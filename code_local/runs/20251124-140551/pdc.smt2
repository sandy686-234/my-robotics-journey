(set-logic QF_LRA)
(set-option :produce-unsat-cores true)
; window 0: [0.022240458, 17.022240458]  lhs=14.839724  rhs=17.000000
(assert (! (<= 14.839724354 17.000000000) :named w0))
; window 1: [0.022240458, 34.022240458]  lhs=30.977823  rhs=34.000000
(assert (! (<= 30.977822585 34.000000000) :named w1))
; window 2: [0.022240458, 54.022240458]  lhs=48.567547  rhs=54.000000
(assert (! (<= 48.567546939 54.000000000) :named w2))
(check-sat)
(get-unsat-core)