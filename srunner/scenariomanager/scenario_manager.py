#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementation.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import sys
import time
import pathlib
import py_trees
from srunner.autoagents.agent_wrapper import AgentWrapper
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.result_writer import ResultOutputProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog
from datetime import datetime
import mss
import cv2
import numpy as np
import json

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent.parent.absolute()) + "/pyproto")

from pyproto import certitrace_pb2
from pyproto import osi_groundtruth_pb2


class AdditionalRunAuditData:
    def __init__(self,queryString, queryURL,certiCAVCommit, carlaCommit, organisation,scenario):
        self.QueryString = queryString
        self.QueryURL = queryURL
        self.CertiCAVCommit = str(certiCAVCommit)
        self.CarlaCommit = carlaCommit
        self.Organisation = organisation
        self.MusiccID = scenario['id']
        pass

class ScenarioManager(object):

    """
    Basic scenario manager class. This class holds all functionality
    required to start, and analyze a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. Trigger a result evaluation with manager.analyze_scenario()
    5. If needed, cleanup with manager.stop_scenario()
    """

    def __init__(self, musiccScenario, queryString, queryURL, concreteScenarioIdentifier, certiCAVCommit, organisation, outputFileName ,debug_mode=False, sync_mode=False, timeout=2.0):
        """
        Setups up the parameters, which will be filled at load_scenario()

        """
        self.scenario = None
        self.scenario_tree = None
        self.scenario_class = None
        self.ego_vehicles = None
        self.other_actors = None
        self.certiTrace = certitrace_pb2.SimulationRecords()

        self._debug_mode = debug_mode
        self._agent = None
        self._sync_mode = sync_mode
        self._running = False
        self._timestamp_last_run = 0.0
        self._timeout = timeout
        self._watchdog = Watchdog(float(self._timeout))

        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None

        self.musiccScenario = musiccScenario
        self.queryString = queryString
        self.queryURL = queryURL
        self.concreteScenarioIdentifier = concreteScenarioIdentifier
        self.certiCAVCommit = certiCAVCommit
        self.organisation = organisation
        self.outputFileName = outputFileName
        self.carlaCommit = None

    def _reset(self):
        """
        Reset all parameters
        """
        self._running = False
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        GameTime.restart()

    def cleanup(self):
        """
        This function triggers a proper termination of a scenario
        """

        if self.scenario is not None:
            self.scenario.terminate()

        if self._agent is not None:
            self._agent.cleanup()
            self._agent = None

        CarlaDataProvider.cleanup()

    def load_scenario(self, scenario, agent=None):
        """
        Load a new scenario
        """
        self._reset()
        self._agent = AgentWrapper(agent) if agent else None
        if self._agent is not None:
            self._sync_mode = True
        self.scenario_class = scenario
        self.scenario = scenario.scenario
        self.scenario_tree = self.scenario.scenario_tree
        self.ego_vehicles = scenario.ego_vehicles
        self.other_actors = scenario.other_actors

        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

        if self._agent is not None:
            self._agent.setup_sensors(self.ego_vehicles[0], self._debug_mode)

    def function_handler(self,event):
        try:
            collision = self.certiTrace.groundTruthSeries[len(self.certiTrace.groundTruthSeries) - 1].collisions.add()
            collisionActor = collision.actors.add()
            otherCollisionActor = collision.actors.add()

            collisionActor.id.value = event.actor.id
            collisionActor.base.position.x = event.actor.get_transform().location.x
            collisionActor.base.position.y = event.actor.get_transform().location.y
            collisionActor.base.position.z = event.actor.get_transform().location.z
            collisionActor.base.orientation.roll = event.actor.get_transform().rotation.roll
            collisionActor.base.orientation.pitch = event.actor.get_transform().rotation.pitch
            collisionActor.base.orientation.yaw = event.actor.get_transform().rotation.yaw
            collisionActor.base.velocity.x = event.actor.get_velocity().x
            collisionActor.base.velocity.y = event.actor.get_velocity().y
            collisionActor.base.velocity.z = event.actor.get_velocity().z
            collisionActor.base.acceleration.x = event.actor.get_acceleration().x
            collisionActor.base.acceleration.y = event.actor.get_acceleration().y
            collisionActor.base.acceleration.z = event.actor.get_acceleration().z

            otherCollisionActor.id.value = event.other_actor.id
            otherCollisionActor.base.position.x = event.other_actor.get_transform().location.x
            otherCollisionActor.base.position.y = event.other_actor.get_transform().location.y
            otherCollisionActor.base.position.z = event.other_actor.get_transform().location.z
            otherCollisionActor.base.orientation.roll = event.other_actor.get_transform().rotation.roll
            otherCollisionActor.base.orientation.pitch = event.other_actor.get_transform().rotation.pitch
            otherCollisionActor.base.orientation.yaw = event.other_actor.get_transform().rotation.yaw
            otherCollisionActor.base.velocity.x = event.other_actor.get_velocity().x
            otherCollisionActor.base.velocity.y = event.other_actor.get_velocity().y
            otherCollisionActor.base.velocity.z = event.other_actor.get_velocity().z
            otherCollisionActor.base.acceleration.x = event.other_actor.get_acceleration().x
            otherCollisionActor.base.acceleration.y = event.other_actor.get_acceleration().y
            otherCollisionActor.base.acceleration.z = event.other_actor.get_acceleration().z
        except Exception as e:
            print(e)
            

    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        print("ScenarioManager: Running scenario {}".format(self.scenario_tree.name))
        self.start_system_time = time.time()
        start_game_time = GameTime.get_time()

        self._watchdog.start()
        self._running = True
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter((self.outputFileName +".avi"), fourcc, 20, 
                            (2560, 1440))
        monitor = {"top": 0, "left": 0, "width": 2560, "height": 1440}
        count = 0
        
        self.carlaCommit = str(CarlaDataProvider.get_client().get_client_version())
        
        while self._running:
            timestamp = None
            world = CarlaDataProvider.get_world()

            
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
                    if count == 0:

                        # Get Weather Conditions
                        # Conversion is from 0-100 down to the osi enum for precipitation 2-8 from none to heavy (0 being unknown  and 1  being other so we add 2*(100/7) to ensure the value is never 0 or 1)
                        self.certiTrace.staticSimulationInformation.environmental_conditions.precipitation = int(world.get_weather().precipitation + 2*(100/7)/ (100/7)) 
                        if (world.get_weather().fog_falloff < 3 and world.get_weather().fog_distance > 1):
                            # Conversion is from 0-100 down to the osi enum for precipitation 1-9 from none to Dense (0 being unknown so we + 1)
                            self.certiTrace.staticSimulationInformation.environmental_conditions.fog = int((world.get_weather().fog_density + 2*(100/8))/ (100/8))
                        else:
                            self.certiTrace.staticSimulationInformation.environmental_conditions.fog = 0;        
                        
                        bluePrintLibrary = world.get_blueprint_library()

                        #Get Bounding Box Info
                        for actor_snapshot in snapshot: 
                            try:
                                actual_actor = world.get_actor(actor_snapshot.id)      
                                try:
                                    isVehicle = int(actual_actor.attributes["number_of_wheels"]) > 0
                                    if(isVehicle):
                                        if actual_actor.attributes['role_name'].lower() == "ego" or actual_actor.attributes['role_name'].lower() == "hero":
                                            print("Ego found")
                                            self.certiTrace.staticSimulationInformation.host_vehicle_id.value = actor_snapshot.id
                                            collision_sensor = world.spawn_actor(bluePrintLibrary.find('sensor.other.collision'), actor_snapshot.get_transform(), attach_to=actual_actor)
                                            collision_sensor.listen(lambda event: self.function_handler(event))
                                        new_static_actor = self.certiTrace.staticSimulationInformation.moving_object.add()
                                        new_static_actor.id.value = actual_actor.id
                                        vehicle = world.get_actors().find(actor_snapshot.id)
                                        new_static_actor.base.dimension.length = vehicle.bounding_box.extent.x * 2
                                        new_static_actor.base.dimension.width = vehicle.bounding_box.extent.y * 2
                                        new_static_actor.base.dimension.height = vehicle.bounding_box.extent.z * 2
                                        if(bluePrintLibrary.find(actual_actor.type_id).has_tag("vehicle")):
                                            new_static_actor.type = 2
                                        elif(bluePrintLibrary.find(actual_actor.type_id).has_tag("pedestrian")):
                                            new_static_actor.type = 3
                                except Exception as e:
                                    print(e)
                                    continue
                                
                            except Exception as e:
                                print(e)
                                print("Error while trying to read static actor info")
                        
                    groundTruth = self.certiTrace.groundTruthSeries.add()
                
                    #Get the actor and the snapshot information
                    for actor_snapshot in snapshot: 
                        new_actor = groundTruth.moving_object.add()
                        new_actor.id.value = actor_snapshot.id
                        new_actor.base.position.x = actor_snapshot.get_transform().location.x
                        new_actor.base.position.y = actor_snapshot.get_transform().location.y
                        new_actor.base.position.z = actor_snapshot.get_transform().location.z
                        new_actor.base.orientation.roll = actor_snapshot.get_transform().rotation.roll
                        new_actor.base.orientation.pitch = actor_snapshot.get_transform().rotation.pitch
                        new_actor.base.orientation.yaw = actor_snapshot.get_transform().rotation.yaw
                        new_actor.base.velocity.x = actor_snapshot.get_velocity().x
                        new_actor.base.velocity.y = actor_snapshot.get_velocity().y
                        new_actor.base.velocity.z = actor_snapshot.get_velocity().z
                        new_actor.base.acceleration.x = actor_snapshot.get_acceleration().x
                        new_actor.base.acceleration.y = actor_snapshot.get_acceleration().y
                        new_actor.base.acceleration.z = actor_snapshot.get_acceleration().z
                    # print(groundTruth)
                    # print("-----------------------------")
            if timestamp:
                try:
                    #take screenshot at each step and store for later writing
                    # with mss.mss() as sct:
                    #     screenshot = np.array(sct.grab(monitor))
                    # screenshot = cv2.resize(screenshot, (2560, 1440))
                    # screenshot = cv2.cvtColor(screenshot, cv2.COLOR_RGB2BGR)
                    # screenshot = cv2.cvtColor(screenshot, cv2.COLOR_BGR2RGB)
                    # out.write(screenshot)
                    print("Step {}".format(str(count)),end="\r")
                    self._tick_scenario(timestamp)
                    count += 1
                except:
                    print("Warning : Failed to tick scenario") 
                    break
        print("")
        out.release()
        collision_sensor.destroy()
            
        self.outputFileName += ".txt"
        self.certiTrace.groupingScenarioIdentifier = self.musiccScenario['metadata']["OpenScenario_ID"]
        self.certiTrace.groupingScenarioDescription = self.musiccScenario['metadata']["Description"]
        self.certiTrace.concreteScenarioIdentifier = self.concreteScenarioIdentifier

        additionalRunAuditDataObject = AdditionalRunAuditData(self.queryString,self.queryURL,self.certiCAVCommit,self.carlaCommit,self.organisation,self.musiccScenario)
        self.certiTrace.additionalRunAuditData = json.dumps(additionalRunAuditDataObject.__dict__)
    
        outputFile = open(self.outputFileName, "wb")
        outputFile.write(self.certiTrace.SerializeToString())
        outputFile.close()

        self._watchdog.stop()

        self.cleanup()

        self.end_system_time = time.time()
        end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - \
            self.start_system_time
        self.scenario_duration_game = end_game_time - start_game_time

        if self.scenario_tree.status == py_trees.common.Status.FAILURE:
            print("ScenarioManager: Terminated due to failure")

    def _tick_scenario(self, timestamp):
        """
        Run next tick of scenario and the agent.
        If running synchornously, it also handles the ticking of the world.
        """

        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            self._watchdog.update()

            if self._debug_mode:
                print("\n--------- Tick ---------\n")

            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()

            if self._agent is not None:
                ego_action = self._agent()

            if self._agent is not None:
                self.ego_vehicles[0].apply_control(ego_action)

            # Tick scenario
            self.scenario_tree.tick_once()

            if self._debug_mode:
                print("\n")
                py_trees.display.print_ascii_tree(self.scenario_tree, show_status=True)
                sys.stdout.flush()

            if self.scenario_tree.status != py_trees.common.Status.RUNNING:
                self._running = False

        if self._sync_mode and self._running and self._watchdog.get_status():
            CarlaDataProvider.get_world().tick()

    def get_running_status(self):
        """
        returns:
           bool:  False if watchdog exception occured, True otherwise
        """
        return self._watchdog.get_status()

    def stop_scenario(self):
        """
        This function is used by the overall signal handler to terminate the scenario execution
        """
        self._running = False

    def analyze_scenario(self, stdout, filename, junit, json):
        """
        This function is intended to be called from outside and provide
        the final statistics about the scenario (human-readable, in form of a junit
        report, etc.)
        """

        failure = False
        timeout = False
        result = "SUCCESS"

        if self.scenario.test_criteria is None:
            print("Nothing to analyze, this scenario has no criteria")
            return True

        for criterion in self.scenario.get_criteria():
            if (not criterion.optional and
                    criterion.test_status != "SUCCESS" and
                    criterion.test_status != "ACCEPTABLE"):
                failure = True
                result = "FAILURE"
            elif criterion.test_status == "ACCEPTABLE":
                result = "ACCEPTABLE"

        if self.scenario.timeout_node.timeout and not failure:
            timeout = True
            result = "TIMEOUT"

        output = ResultOutputProvider(self, result, stdout, filename, junit, json)
        output.write()

        return failure or timeout
