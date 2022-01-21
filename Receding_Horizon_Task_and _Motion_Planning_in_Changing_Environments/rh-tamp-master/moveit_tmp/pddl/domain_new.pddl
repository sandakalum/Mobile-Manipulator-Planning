(define (domain manipulation)
(:requirements :adl)
(:types movable fixed point - object)
(:predicates (arm-empty)
             (holding ?o - object)
             (obstruct ?a - object ?b - object)
             (on ?a - object ?b - object)
             (free-surface ?o - object)
)
(:action pickup
  :parameters (?m - movable)
  :precondition
  (and
    (arm-empty)
    (forall (?o - object) (not (obstruct ?o ?m)))
  )
  :effect
  (and
    (holding ?m)
    (not (arm-empty))
  )
)
(:action putdown
  :parameters (?m - movable ?s - object)
  :precondition (and (holding ?m) (free-surface ?s) (not (= ?m ?s)))
  :effect
  (and
    (arm-empty)
    (not (holding ?m))
    (on ?m ?s)
    (forall (?o - object) (not (obstruct ?m ?o)))
    (forall (?o - object)
      (when (on ?m ?o)
            (free-surface ?o)
      )
    )
    (forall (?o - object) (not (on ?m ?o)))
  )
)
)
