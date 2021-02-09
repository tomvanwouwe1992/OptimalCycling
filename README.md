# OptimalCycling
Optimize subject-specific bicycle parameters to optimize cycling performance

Bike-fitting can benefit from model based simulations. Depending on subject-specific anthropometry, flexibility, relative strength, etc. bike parameters need to be adapted to optimize cycling performance. Even the cycling task at hand (e.g. short intense time-trial, Iron Man bike leg, long climb, ...) requires different positions to optimize performance.
Here we start with a simulation based framework that aims to bring as much parameters as possible into account. This type of tool could be applied to inform users on what their optimal position should be; but more importantly a simulation framework allows an extensive analysis and understanding of cause-effect relations. Therefore it can yield insights in many different factors that might not even have been considered.




STEP 1: Have a torque driven multi-segment model for which we can specify anthropometry. Given a speed, find the optimal position (saddle, handlebars, pedal axles, crank length) and torque actuator coordination that minimizes effort throughout a revolution. We use a very simple model of air resistance that depends on the angle of the trunk.
STEP 2: Include passive structures that impose (realistic) bounds on the attainable positions.
STEP 3: ...
