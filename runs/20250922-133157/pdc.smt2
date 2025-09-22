(set-logic QF_LRA)
(set-option :produce-unsat-cores true)
; window 0: [2408.234115473, 2425.234115473]  lhs=16.188790  rhs=17.000000
(assert (! (<= 16.188790205 17.000000000) :named w0))
; window 1: [2408.234115473, 2442.234115473]  lhs=33.793988  rhs=34.000000
(assert (! (<= 33.793988275 34.000000000) :named w1))
; window 2: [2408.234115473, 2462.234115473]  lhs=52.982778  rhs=54.000000
(assert (! (<= 52.982778479 54.000000000) :named w2))
(check-sat)
(get-unsat-core)