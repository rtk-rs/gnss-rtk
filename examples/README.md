Examples
========

Example applications. All of these examples serve two purposes:

- they will teach you how to deploy the position Solver.   
- they serve as a complete Testers of the library and are fully integrated to our continuous integration process.  
Therefore, it's possible to modify them for your own learning process, but any proposed modification will have to be studied.

Therefore, in input data here are well known and we expect a certain level of performance to be sustained.

Solver Deployment
=================

You need three steps to deploy the Solver:

- [x] define a Configuration setup
- [x] Tie an Orbital state provider to the solver. This 
typically comes from an Ephemeride source, but not always
- [x] Tie a data source that will be able to consume

In RTK you need one to fulfill one extra step
- [x] Tie a Base station to the Solver.

By consuming the data source, solutions will start to appear.  
Navigation is a complex iterative process, esspecially in PPP mode.  
Therefore, it is normal to perform somewhat badly in the first few iterations.  
It is also normal to not be able to resolve from time to time.   
You may also have difficulty to initialize the process (prepare for first fruitful iteration) 
depending on the quality of your data or the global context.  

GNSS-RTK needs at best two iterations to generate something.  
Say your first iteration is good enough for the solver to initialize itself, we will return
the special `FirstSolution` cause of solution invalidation.

When performing a survey without apriori knowledge (worst case scenario), the initial requirements
are more demanding. You need more 4 SV in sight, whatever your navigation technique. Therefore
it is even more demanding to arm the solver in that scenario. As long as the solver is not armed,
it will not be able to produce any solutions. When arming process is in failure, we return
the special `FirstGuess` Error. If arming is not even attempted, due to lack of input data,
we return the other Errors that indicate that kind of problem.
