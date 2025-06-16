# Project

## Goals

### Model based controller

### Data driven controller

## Provided Material

- Linear Model

- Time series data

## Notebook

- challenging morning commute evaluation scenario
- SUMO Traffic simulator
- P Controller
- macroscopic flow diagramm (used to determine optimal densities)

### main.ipynb

#### Setup

- don't touch
- (mfd\_)taskparams?

#### Traffic Inflow Problem

##### Grid

- 5 regions, 0-3 periphery region, 4 city-center
- **Dynamic Speed Limits** [0.5,1.5]

Coco City Algorithm
Input

- Density of car per road
- Forecast of cars that will spawn in each region
  Output
- DSL

### coco provides

#### SUMO

- Simulation time step 20 seconds
- Task init creats simulation
- DSL-Task creates ControlSim with functionality to run DSL simulation

#### Optimal densities for each region

- Macroscopic Fundamental Diagram (MFD)
- optimal density $\rho^\star$ immutable?

#### Training data

Gathered from offline traffic simulations with simple P controller

Simulation provides

- density of car in each region
- number of cars spawning in each region
- underlying flow not provided
- plot function

#### Linear traffic model

$$\rho(k+1) = A\rho(k)+Bv(k)+Cq(k)+d$$

- DSL ratio $v$ and spawned vehicles $q$
- Nominal linearization point $v^\star=1$ and $\rho^\star$ as above
- Original dynamics linear in $q$ therefore no linearization point $q^\star$

#### Evaluation scenario

- Perfect prediction of vehicle spawn given
- plot function

#### Controller implementation

- DSL Task creates simulation object `ControlSim`
- Simulation starts with `dsl_task.runtask()`
- `ControlSim` must include function `compute_input()`
- The `ControlClass` must include function `get_next_input()`
- `get_next_input()` for implementation of computational controller
- Evaluate controller `experiment = dsl_task.runtask()` with evaluation dataset

#### Defining control parameters

- When instantiate controller, pass parameters ad dictionary
- Includes f.e. control cost and regularization parameters
- May also include dynamic matrices or f.e. Hankel matrix

#### Controller evaluation tools

- `plot_density()`
- `plot_flow()`
- `plot_input()`
- `plot_metrics()`

### Coco current status P

## Task

### 5 Slide Presentation

Die Präsentation ist für den Bürgermeister von COCO City und die Abteilung Verkehrskontrolle bestimmt. Sie muss überzeugend, prägnant, grafisch ansprechend und professionell sein. Sie darf nicht zu technisch sein, aber sie muss eine überzeugende logische Abfolge haben und wissenschaftlich fundiert sein.

#### Slide 1 (0/1)(0/0)

- **Current state**
- show performance of current no control
- briefly comment on the behaviour

#### Slide 2 (0/2)(0/0)

- **Failure mode**
- show how P controller can fail evaluation task
- show why is new computational controller (not a) good idea

#### Slide 3 (0/4)(0/0)

- **Your model-based recommendation + Demonstration**
- Explain what type of model-based controller you would recommend
- provide the three most important reasons based on the characteristics of the controller
- demonstrate how the model-based controller compares with the P controller

#### Slide 4 (0/4)(0/0)

- **Your data-driven recommendation + Demonstration**
- Explain what type of data-driven controller you would recommend, given their dataset
- provide the three most important reasons based on the characteristics of the controller
- demonstrate how the data-driven controller compares with the P controller

#### Slide 5 (0/4)(0/0)

- **Plan for deployment**
- Describe necessary technical steps to deploy the 2 controllers
- Describe most important requirements and trade-offs between the 2 approaches

#### Slide 6 (0/0)(0/2)

- **Extra Slide**
- Build a bonus data-driven controller and compare with other 2
- present the requirements, merits, and trade-offs of your control design in an extra
  (sixth) slide

### Jupyter Notebook

#### Execution (0/5)(0/2)

- run under 5 minutes
- must produce material for the slides

#### Clear code implementation (0/5)(0/0)

- Clear comments when necessary
- defining all variables, functions

#### ReadMe cell at the start of the notebook (0/5)(0/1)

- Model-based
  - How the proposed controller is implemented.
  - What **parameters need to be tuned**
  - How you tuned and how you would recommend tuning the parameters
- Data-driven
  - How the proposed controller is implemented.
  - What **parameters need to be tuned**
  - How you tuned and how you would recommend tuning the parameters
- Extra slide
  - Clear explanation of the implementation considerations
