(define (domain manipulation)
 
(:types obstacle robot location)  

(:predicates    
             (at ?rob ?from)
             (handEmpty)
	     (holding ?rob ?obs)
             (in ?obs ?from))
            
(:action move
:parameters (?rob - robot ?from - location ?to - location)
:precondition (at ?rob ?from)           
:effect (and (at ?rob ?to) 
   (not (at ?rob ?from))))

(:action pick
:parameters (?rob - robot ?obs - obstacle ?from - location)
:precondition (and (handEmpty) (in ?obs ?from) 
                   (at ?rob ?from))          
:effect (and (holding ?rob ?obs) 
   (not (handEmpty)) ))

(:action place
:parameters (?rob - robot ?obs - obstacle ?from - location)
:precondition (and (holding ?rob ?obs) 
                   (at ?rob ?from))          
:effect (and (handEmpty) (in ?obs ?from) 
   (not (holding ?rob ?obs)) ))

)






