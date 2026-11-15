# Bench2Drive's 44 scenarios

Bench2Drive's 220 routes (`external/simlingo/leaderboard/data/bench2drive_split/bench2drive_NN.xml`) are 44 scenario
types, 5 routes each. The scenarios run from Bench2Drive's own scenario_runner
(`external/simlingo/Bench2Drive/scenario_runner/srunner/scenarios/`, the one `scripts/run_bench2drive.sh` puts on the
path); the descriptions below are from that code, not only its docstrings (several are copied from other scenarios and
wrong: EnterActorFlow, HighwayCutIn and CrossingBicycleFlow all say "another vehicle runs a red light"). Every route
also has the background traffic (BackgroundActivity) and is scored on route completion and infractions; the scenario
is what is placed along it.

"Tried" marks the types this stack has been developed on (routes 0, 1, 4, 6, 8, 10); the rest have never been driven
by simlingo_nmpc. List made 15 Nov 2026.

## Obstacles in the ego's lane: go round

| Type | Routes | Towns | What happens | Asks of the stack | Tried |
|---|---|---|---|---|---|
| ParkedObstacle | 1, 34, 156, 165, 169 | 05, 06, 12, 15 | A car parked badly, partly in the ego's lane, on a multi-lane road. | Change into the neighbouring lane going the same way and back. | route 1 |
| ParkedObstacleTwoWays | 37, 38, 72, 176, 177 | 03, 11, 12, 13 | As ParkedObstacle on a road with one lane each way; oncoming traffic comes at `frequency`. | Borrow the oncoming lane in a gap, back in after. | no |
| ConstructionObstacle | 30, 31, 155, 160, 161 | 04, 06, 12 | Cones and a warning sign closing the ego's lane, multi-lane road. | Change lane round the works. | no |
| ConstructionObstacleTwoWays | 4, 5, 35, 127, 170 | 11, 12 | As ConstructionObstacle, one lane each way, oncoming flow. | Borrow the oncoming lane in a gap. | route 4 |
| Accident | 32, 33, 65, 148, 162 | 03, 10HD, 12, 13 | A police car stopped before two crashed cars in the ego's lane, multi-lane road. | Change lane round them. | no |
| AccidentTwoWays | 6, 36, 70, 172, 174 | 05, 12, 13, 15 | As Accident, one lane each way, oncoming flow. | Borrow the oncoming lane in a gap. | route 6 |
| HazardAtSideLane | 2, 3, 164, 168, 171 | 04, 05, 12, 15 | Three slow bicycles riding at the edge of the ego's lane, road without sidewalk. | Pass them leaving room, or follow. | route 2 (100 %, 15 Nov) |
| HazardAtSideLaneTwoWays | 71, 150, 173, 179, 180 | 03, 04, 05, 07, 13 | As HazardAtSideLane, one lane each way, oncoming flow. | Borrow the oncoming lane round the bicycles in a gap. | no |
| VehicleOpensDoorTwoWays | 73, 74, 75, 76, 178 | 11, 13 | A car parked at the side opens its door into the ego's lane; one lane each way. | Swerve or borrow the oncoming lane round the door. | no |
| BlockedIntersection | 20, 21, 49, 200, 204 | 04, 11, 12 | The ego turns at a junction and finds the exit blocked by a stopped car, removed after a while. | Stop behind it in the junction, wait, go on. | no |
| InvadingTurn | 41, 42, 81, 82, 83 | 12, 13 | Turning right on a one-lane-each-way road, oncoming cars come over into the ego's lane (cones on their side). | Keep right, off the line, without stopping. | no |

## Cars cutting in or stopping ahead

| Type | Routes | Towns | What happens | Asks of the stack | Tried |
|---|---|---|---|---|---|
| ParkingCutIn | 0, 124, 125, 126, 157 | 05, 12 | A car parked at the side pulls out into the ego's lane just ahead. | Brake for it, follow it. | route 0 |
| StaticCutIn | 39, 40, 166, 185, 187 | 05, 06, 12, 15 | A car stopped in the side lane (a queue there) cuts in ahead. | Brake for it, follow it. | no |
| HighwayCutIn | 24, 51, 52, 53, 96 | 12, 13 | On a highway a car comes in from an entry ramp and cuts into the ego's lane 10 m ahead at 80 % of its speed. | Brake, keep a gap. | no |
| HardBreakRoute | 79, 152, 158, 188, 192 | 01, 03, 04, 10HD, 13 | The background cars ahead of the ego all brake hard to a stop, wait, then go on. | Follow at a gap that can stop in time. | route 79 (100 %, 15 Nov) |
| ParkingExit | 7, 77, 78, 183, 191 | 03, 12, 13 | The ego starts parked in the parking lane between two cars and must pull out into the driving lane (lane test off until 40 m on). | Leave a tight space and merge into traffic from rest. | no |

## Junctions: turning across or into flows

| Type | Routes | Towns | What happens | Asks of the stack | Tried |
|---|---|---|---|---|---|
| SignalizedJunctionLeftTurn | 103, 104, 105, 106, 107 | 12 | Left at a light, a flow of oncoming cars (`flow_speed`, gaps `source_dist_interval`). | Wait at green for a gap in the oncoming flow, cross. | route 103 (26.9 %: collision mid-crossing, 15 Nov) |
| NonSignalizedJunctionLeftTurn | 10, 11, 12, 45, 198 | 03, 12 | Left with no light (stop sign), oncoming flow. | Stop, take a gap across the flow. | route 10 (100 % once, 15 Nov) |
| SignalizedJunctionLeftTurnEnterFlow | 210, 213, 217, 218, 219 | 04, 12, 15 | Left at a light; the oncoming flow turns right into the road the ego turns into. | Cross and enter that flow in a gap. | no |
| NonSignalizedJunctionLeftTurnEnterFlow | 208, 209, 214, 215, 216 | 04, 12 | As above without a light. | As above, after the stop. | no |
| SignalizedJunctionRightTurn | 8, 108, 196, 197, 199 | 05, 12, 15 | Right at a light into a lane with a flow of cars from the left. | Merge into the flow in a gap at green. | route 8 |
| NonSignalizedJunctionRightTurn | 13, 46, 84, 85, 86 | 12, 13 | As above without a light. | Merge into the flow after the stop. | no |
| InterurbanActorFlow | 136, 137, 138, 139, 144 | 12, 13 | Leaving an interurban road by a left turn across a flow of oncoming cars. | Wait in the road for a gap, cross. | no |
| InterurbanAdvancedActorFlow | 132, 133, 134, 141, 142 | 12, 13 | Joining an interurban road by a left turn: cross one flow, then merge into the other. | Two gaps in a row. | no |
| EnterActorFlow | 19, 93, 111, 112, 201 | 05, 12, 13 | A junction where the ego joins a road with a continuous flow of cars. | Merge into the flow in a gap. | no |
| OppositeVehicleRunningRedLight | 9, 43, 44, 194, 195 | 03, 04, 12 | The ego has green; a car from the side runs its red across the junction. | See it and brake or let it pass. | no |
| OppositeVehicleTakingPriority | 14, 15, 16, 47, 87 | 12, 13 | Unsignalised junction; a car from the side goes across without yielding. | Yield to it though it has no right. | route 14 (100 %, 15 Nov) |
| CrossingBicycleFlow | 54, 55, 56, 57, 58 | 12 | The ego turns across a bike lane with a flow of bicycles (their light held green). | Wait for a gap in the bicycles, cross. | no |

## Pedestrians and cyclists crossing

| Type | Routes | Towns | What happens | Asks of the stack | Tried |
|---|---|---|---|---|---|
| DynamicObjectCrossing | 122, 146, 147, 149, 153 | 01, 02, 11, 12, 15 | A pedestrian or cyclist steps out from behind a static blocker (`blocker_model`) and crosses. | See it late, brake hard. | route 122 (100 %, 15 Nov) |
| ParkingCrossingPedestrian | 63, 64, 123, 145, 151 | 02, 03, 12, 13 | As above, the blocker is a parked car. | Brake for the pedestrian appearing from between cars. | no |
| PedestrianCrossing | 113, 175, 202, 203, 205 | 03, 04, 07, 11, 12 | A group of pedestrians crosses the road at the junction ahead. | Stop and wait for all of them. | no |
| VehicleTurningRoute | 17, 48, 88, 89, 90 | 12, 13 | After the ego turns at a junction, a cyclist crosses its new road. | Brake for the cyclist just after the turn. | no |
| VehicleTurningRoutePedestrian | 18, 91, 92, 109, 110 | 12, 13 | As above with a pedestrian (the docstring says cyclist; the code spawns a walker). | Brake for the pedestrian after the turn. | no |

## Highway lanes and yielding

| Type | Routes | Towns | What happens | Asks of the stack | Tried |
|---|---|---|---|---|---|
| YieldToEmergencyVehicle | 66, 67, 68, 69, 167 | 03, 13 | An emergency vehicle comes up from behind in the ego's lane on a multi-lane road. | Change lane to let it by (scored: `yield_emergency_vehicle_infractions`). | route 66 (70 %: did not yield, 15 Nov) |
| MergerIntoSlowTraffic | 22, 23, 50, 94, 95 | 12, 13 | At a highway confluence the ego must merge into a slow flow of cars. | Slow down and merge into a slow stream. | no |
| MergerIntoSlowTrafficV2 | 135, 143, 184, 186, 189 | 06, 12, 13 | A variant at another kind of confluence. | As above. | no |
| HighwayExit | 128, 129, 130, 131, 140 | 12, 13 | A fast flow in the lane next to the ego; the ego must cut into it and take the exit. | Speed up, change lane into a fast stream, exit. | no |
| SequentialLaneChange | 117, 118, 119, 120, 121 | 12 | No actors of its own: the route asks for several lane changes in a row through traffic. | Follow the route's lane changes. | no |

## Rules only (no adversary)

| Type | Routes | Towns | What happens | Asks of the stack | Tried |
|---|---|---|---|---|---|
| VanillaSignalizedTurnEncounterRedLight | 25, 59, 97, 98, 99 | 12, 13 | A turn at a light that is red on arrival. | Stop at red, go at green. | route 25 (100 %, 15 Nov) |
| VanillaSignalizedTurnEncounterGreenLight | 114, 115, 116, 181, 182 | 07, 12 | A turn at a light that is green. | Turn without stopping. | no |
| VanillaNonSignalizedTurn | 26, 27, 28, 60, 100 | 12, 13 | A turn at an unsignalised junction. | Turn, yield to traffic. | no |
| VanillaNonSignalizedTurnEncounterStopsign | 29, 61, 62, 101, 102 | 12, 13 | A turn at a stop sign. | Stop fully at the sign, then go. | no |
| T_Junction | 193, 206, 207, 211, 212 | 01, 02, 04, 07, 10HD | No actors of its own: a route through a T junction. | Lights and stop signs, background traffic. | no |
| ControlLoss | 80, 154, 159, 163, 190 | 01, 02, 10HD, 11, 13 | Debris on the road; when the ego drives over it, noise is added to its steering and throttle (loss of grip). | Recover and keep the lane. | no |

## Coverage

15 Nov: one route of seven more types (one per category, two junction ones): 5 at 100 %; route 103
(SignalizedJunctionLeftTurn) collided mid-crossing, route 66 (YieldToEmergencyVehicle) did not yield (nothing in
the stack does yet). Thirteen of 44 types tried.

Six of the 44 types have been driven by simlingo_nmpc (ParkingCutIn, ParkedObstacle, ConstructionObstacleTwoWays,
AccidentTwoWays, SignalizedJunctionRightTurn, NonSignalizedJunctionLeftTurn). By need:

- Obstacles to go round (11 types): tried on 3 (ParkedObstacle, ConstructionObstacleTwoWays, AccidentTwoWays).
- Cut-ins and braking ahead (5 types): tried on 1 (ParkingCutIn).
- Gaps in flows at junctions (12 types): tried on 2 (route 8 right merge, route 10 left across).
- Pedestrians and cyclists (5 types, plus the bicycles of HazardAtSideLane and CrossingBicycleFlow): none.
- Highway lanes, emergency vehicle (5 types): none.
- Rules only (6 types): none as such; lights and stop signs were met on the routes tried.
