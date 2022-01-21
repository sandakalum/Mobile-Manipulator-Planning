(define (domain manipulation)
(:requirements :adl)
(:types item surface - object
        movable fixed - item
)
(:predicates (arm-empty)
             (holding ?i - item)
             (obstruct ?a - item ?b - object)
             (on ?a - item ?b - object)
             (is-stackable ?i - item)
             (leave-clean ?i - item ?m - movable)
)
(:action pickup
  :parameters (?m - movable ?s - surface)
  :precondition
  (and
    (arm-empty)
    (on ?m ?s)
    (not (exists (?i - item) (obstruct ?i ?m)))
    (not (exists (?i - item) (on ?i ?m)))
  )
  :effect
  (and
    (holding ?m)
    (not (arm-empty))
    (forall (?i - item) (not (obstruct ?m ?i)))
    (not (on ?m ?s))
  )
)
(:action unstack
 :parameters (?m - movable ?i - item)
 :precondition
  (and
    (arm-empty)
    (on ?m ?i)
    (not (exists (?o - item) (obstruct ?o ?m)))
    (not (exists (?o - item) (on ?o ?m)))
  )
 :effect
  (and
    (holding ?m)
    (not (arm-empty))
    (forall (?o - item) (not (obstruct ?m ?o)))
    (forall (?s - item) (not (leave-clean ?s ?m)))
    (not (on ?m ?i))
  )
)
(:action putdown
 :parameters (?m - movable ?s - surface)
 :precondition
  (and
    (holding ?m)
    (not (= ?m ?s))
    (not (exists (?i - item) (obstruct ?i ?s)))
  )
 :effect
  (and
    (arm-empty)
    (not (holding ?m))
    (on ?m ?s)
  )
)
(:action stack
 :parameters (?m - movable ?s - item)
 :precondition
  (and
    (holding ?m)
    (not (= ?m ?s))
    (not (exists (?i - item) (on ?i ?s)))
    (not (exists (?i - movable) (leave-clean ?s ?i)))
    (not (exists (?i - item) (obstruct ?i ?s)))
    (is-stackable ?s)
  )
 :effect
  (and
    (arm-empty)
    (not (holding ?m))
    (on ?m ?s)
  )
)
)
