
nationality([N, _, _, _, _], N).
color([_, C, _, _, _], C).
drink([_, _, D, _, _], D).
cigarette([_, _, _, C, _], C).
pet([_, _, _, _, P], P).


middle(X, [_, _, X, _, _]).
leftmost(X, [X, _, _, _, _]).
lateral(X, [X, _, _, _, _]).
lateral(X, [_, _, _, _, X]).

member(X, [X | _]).
member(X, [_ | T]) :- member(X, T).

near(X, Y, [X, Y, _, _, _]).
near(X, Y, [_, X, Y, _, _]).
near(X, Y, [_, _, X, Y, _]).
near(X, Y, [_, _, _, X, Y]).
near(X, Y, [Y, X, _, _, _]).
near(X, Y, [_, Y, X, _, _]).
near(X, Y, [_, _, Y, X, _]).
near(X, Y, [_, _, _, Y, X]).


solve(S) :-
    S = [_, _, _, _, _],
    nationality(X1, englishman), color(X1, red), member(X1, S),
    color(X2, green), drink(X2, coffee), member(X2, S),
    nationality(X3, italian), drink(X3, tea), member(X3, S),
    color(X4, yellow), cigarette(X4, kent), member(X4, S),
    drink(X5, milk), middle(X5, S),
    
    nationality(X6, russian), leftmost(X6, S),
    
    cigarette(XA7, marlboro), pet(XB7, fox), near(XA7, XB7, S),
    cigarette(XA8, kent), pet(XB8, horse), near(XA8, XB8, S),
    drink(X9, juice), cigarette(X9, pall_mall), member(X9, S),
    nationality(X10, japanese), cigarette(X10, assos), member(X10, S),
    
    nationality(XA11, russian), color(XB11, blue), near(XA11, XB11, S),
    nationality(X12, spanish), pet(X12, dog), member(X12, S),
    pet(X13, snake), cigarette(X13, camel), member(X13, S),
    color(XA14, grey), color(XB14, green), near(XA14, XB14, S),
    pet(XA15, cat), drink(XB15, vodka), lateral(XA15, S), lateral(XB15, S).
