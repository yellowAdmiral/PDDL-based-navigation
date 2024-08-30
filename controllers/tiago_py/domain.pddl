(define (domain example)
	(:requirements :typing)
	(:types
		localisable location - object
		room - location
		robot - localisable
	)
	(:predicates
		(in ?obj - localisable ?loc - location)
		(cango ?from - location ?to - location)
	)
	(:action moveto
		:parameters (
			?bot - object
			?from - location
			?to - location
		)
		:precondition (
		and
			(in ?bot ?from)
			(cango ?from ?to)
		)
		:effect (
		and
			(in ?bot ?to)
			(not (in ?bot ?from))
		)
	)
)
