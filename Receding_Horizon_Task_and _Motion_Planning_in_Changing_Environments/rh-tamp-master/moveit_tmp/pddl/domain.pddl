(define (domain can)
(:requirements :strips)
(:predicates (arm-empty)
             (holding ?x - objs)
             (obstruct ?x - objs ?y - objs)
)

(:action pickup
  :parameters (?ob - objs)
  :precondition (and (arm-empty) (forall (?obs - objs) (not (obstruct ?obs ?ob))))
  :effect (and (holding ?ob) (not (arm-empty)))
)

(:action putdown
  :parameters (?ob - objs)
  :precondition (holding ?ob)
  :effect (and (arm-empty) (not (holding ?ob)) (forall (?obj - objs) (not (obstruct ?ob ?obj))))
)
)
