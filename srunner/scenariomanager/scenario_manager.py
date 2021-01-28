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

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent.parent.absolute()) + "/pyproto")

from pyproto import certitrace_pb2
from pyproto import osi_groundtruth_pb2


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

    def __init__(self, musiccScenario, queryString, queryURL, downloadID, certiCAVCommit, organisation, outputFileName, debug_mode=False, sync_mode=False, timeout=2.0):
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
        self.downlaodID = downloadID
        self.certiCAVCommit = certiCAVCommit
        self.organisation = organisation
        self.outputFileName = outputFileName

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

        self.certiTrace.staticSimulationInformation
        while self._running:
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
                    groundTruth = self.certiTrace.groundTruthSeries.add()
                    for actor_snapshot in snapshot: #Get the actor and the snapshot information
                        new_actor = groundTruth.moving_object.add()
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
                    print(groundTruth)
                    print("-----------------------------")
            if timestamp:
                try:
                    #take screenshot at each step and store for later writing
                    with mss.mss() as sct:
                        screenshot = np.array(sct.grab(monitor))
                    screenshot = cv2.resize(screenshot, (2560, 1440))
                    screenshot = cv2.cvtColor(screenshot, cv2.COLOR_RGB2BGR)
                    screenshot = cv2.cvtColor(screenshot, cv2.COLOR_BGR2RGB)
                    out.write(screenshot)

                    self._tick_scenario(timestamp)
                    count += 1
                except:
                    print("Warning : Failed to tick scenario")
                    break
        
        out.release()

        self.outputFileName += ".txt"
            
        outputFile = open(self.outputFileName, "w")

        outputFile.write("---------------------HEADER---------------------\n")
        outputFile.write("Musicc Scenario label : {0}\n".format(self.musiccScenario['metadata']["label"]) + 
        "Musicc Scenario Description : {0}\n".format(self.musiccScenario['metadata']["Description"]) + 
        "Query String : {0}\n".format(self.queryString) + 
        "Query URL : {0}\n".format(self.queryURL) + 
        "Musicc Download ID : {0}\n".format(self.downlaodID) + 
        "CertiCAV Commit : {0}\n".format(self.certiCAVCommit) + 
        "Carla Version : {0}\n".format(CarlaDataProvider.get_client().get_client_version()) + 
        "Organisation : {0}\n".format(self.organisation))

        
        timestepCounter = 0
        for timestep in self.ground_truth_series:
            timestepCounter += 1
            outputFile.write("---------------------{0}---------------------\n".format(timestepCounter))
            actorCounter = 0
            for actor in timestep:
                actorCounter += 1
                outputFile.write("Actor {0} : position X: {1}, position Y: {2}, position Z: {3}\n".format(actorCounter,actor.location.x,actor.location.y,actor.location.z))

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
