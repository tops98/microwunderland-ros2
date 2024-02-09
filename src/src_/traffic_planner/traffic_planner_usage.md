# How to use the traffic planner
The folowing guide contains the minimal set of steps that are necessary to get the traffic planner node up and running.
## Prerequisites
1. Start the *object_detector* node
1. Start the *object_tracker* node
1. Turn on all smart cars that you want to use and make sure the tracking id matches the topic used to connect to the car the *object_tracker* node
1. Start the *servo_controller* node and make sure that the configuration file for the switches is containig the correct informations

## Service Summery
| name | args | returns | description | 
| ---- | ---- | ------- | ----------- |
| AddTravelPlan | **name**: *travel_plan*,<br>**type:** *string*,<br> **value:** *travel plan in yaml format(see example file if you need to know how to structure them structure)* | **type:** *int*,<br> **value:**<br> *0=plan valid and added <br> 1=invalid not added*  | Adds a new travel plan to the list of available travel plans|
| AssigneTravelPlan | **name**: *plan_name*,<br>**type:** *string*,<br> **value:** *name of the travelplan* <hr> **name**: *vehicle_name*,<br>**type:** *string*,<br> **value:** *name of the tracked vehicle that should execute the plan* <hr> **name**: *vehicle_host_name*,<br>**type:** *string*,<br> **value:** *hostname or ip address of the tracked vehicle that should execute the plan* | **type:** *int*,<br> **value:**<br> *0=travel plan assigned <br> 1=not assigned, vehicle not found <br> 2=not assigned, plan not found* | assigns a travel plan to a tracked vehicle |
| Start | **name**: *vehicle_names*,<br>**type:** *string array*,<br> **value:** list with ids of the tracked vehicles| **type:** *int*,<br> **value:**<br> *0=start executing plan <br> 1=vehicle unkown <br> 2=no plan asigned* | starts or resumes the asigned travel plans for all listed vehicles |
| Pause | **name**: *vehicle_names*,<br>**type:** *string array*,<br> **value:** list with ids of the tracked vehicles| **type:** *int*,<br> **value:**<br> *0=pause executing plan <br> 1=vehicle unkown <br> 2=no plan asigned* | pauses the assigned travel plans for all listed vehicles |
| Stop | **name**: *vehicle_names*,<br>**type:** *string array*,<br> **value:** list with ids of the tracked vehicles| **type:** *int*,<br> **value:**<br> *0=stop executing plan <br> 1=vehicle unkown <br> 2=no plan asigned* | Stop the asigned travel plans for all listed vehicles and reset state |


## Step by step guid
1. Make sure the launch file contains a path to a valid map location in the ***map_path*** parameter and that the maps data is correct
1. If you have a file containig travell plans you can specify its location in the ***travel_plan_path*** argument in the launch file or set in durring lauch of the node using the --ros-args -p traffic_plan_path:=<path to travel plans\> option. If you want to add a travel plan after start up you can do so by calling the ***AddTravelPlan Service***
1. Once the node has launched you can asigne a travel plan to a tracked object of your choice by calling the ***AssigneTravelPlan Service***
1. After asigning the travel plans (be aware a tracked object can alway only have one travel plan asigned to it) you can select which vehicles should start executing a or resume executing there asigned travel plan using the ***Start Service***  
1. To pause the execution of a travel plan on a tracked object you can call the ***Pause Service*** or to pause every plan that is in execution call the ***PauseAll Service***
1. In order to stop the execution of a travel plan on a tracked object you can call the ***Stop Service*** or to stop every plan that is in execution call the ***StopAll Service***