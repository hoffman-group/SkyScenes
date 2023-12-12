import sys
sys.path.append('PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg')
import argparse
import carla
import cv2
import json
import numpy as np
import os
import queue
import random
import time 
from PIL import Image

np.random.seed(0)
random.seed(0)

'''Color Dictionary for carla Simulartor 0.9.14'''
carla_colordict_14 = {
                0: (0, 0, 0),                       # Unlabeled
                1: (70, 70, 70),                    # Building  
                2: (100, 40, 40),                   # Fence
                3: (55, 90, 80),                    # Other
                4: (220, 20, 60),                   # Pedestrian
                5: (153, 153, 153),                 # Pole
                6: (157, 234, 50),                  # RoadLine
                7: (128, 64, 128),                  # Road
                8: (244, 35, 232),                  # SideWalk
                9: (107, 142, 35),                  # Vegetation
                10: (0, 0, 142),                    # Vehicles -- Cars, vans, trucks, motorcycles, bikes, buses, trains. (old car class)
                11: (102, 102, 156),                # Wall
                12: (220, 220, 0),                  # TrafficSign
                13: (70, 130, 180),                 # Sky
                14: (81, 0, 81),                    # Ground
                15: (150, 100, 100),                # Bridge
                16: (230, 150, 140),                # RailTrack            
                17: (180, 165, 180),                # GuardRail
                18: (250, 170, 30),                 # TrafficLight
                19: (110, 190, 160),                # Static  [new class!]
                20: (170, 120, 50),                 # Dynamic [new class!]
                21: (45, 60, 150),                  # Water
                22: (145, 170, 100),                # Terrain (old low vegetation class)
                } 


class genImages(object):
    def __init__(self, args):
        self.ROOT_DIR = args.ROOT_DIR
        print(f"Saving DIR:     {os.path.join(self.ROOT_DIR, f'H_{int(args.height)}_P_{abs(int(args.pitch))}/{args.weather}/{args.town}')}")
        print("__"*20)
        ### Internal arguments
        self.IMG_WIDTH = 2160
        self.IMG_HEIGHT = 1440
        self.SIGMA_H = 2.5
        self.SIGMA_P = 5
        
        self.FOV = 110      # Does not change
        self.SENSOR_X = 5   # Location of sensor
        self.SENSOR_Z = 2.5 # Location of sensor
        self.port = 2000
        self.carlaClassNum = 8 
        
        ### External arguments
        self.town = args.town
        self.height = args.height
        self.heightCamera = np.random.normal(int(self.height), self.SIGMA_H)
        self.pitch = args.pitch
        self.pitchCamera = np.random.normal(int(self.pitch), self.SIGMA_P)
        self.totalImages = int(args.num)
        self.weather_str = args.weather
        if args.weather == "ClearNoon":
            self.weather = carla.WeatherParameters.ClearNoon
        elif args.weather == "CloudyNoon":
            self.weather = carla.WeatherParameters.CloudyNoon
        elif args.weather == "MidRainyNoon":
            self.weather = carla.WeatherParameters.MidRainyNoon
        elif args.weather == "ClearSunset":
            self.weather = carla.WeatherParameters.ClearSunset
        elif args.weather == "ClearNight":
            self.weather = carla.WeatherParameters(
                                                    cloudiness = 0.0,
                                                    precipitation = 0.0,
                                                    precipitation_deposits = 0.0,
                                                    wind_intensity = 0.0,
                                                    sun_azimuth_angle = -1.0,
                                                    sun_altitude_angle = -90.0, 
                                                    fog_density = 0.0,
                                                    fog_distance = 0.0, 
                                                    wetness = 0.0
                                                    )
        
        self.vehiclesSpawned = 0 # number of vehicles
        self.walkerSpawned = 0   # number of walkers 

        ### Creating Directories
        ## Save Data
        os.makedirs(os.path.join(self.ROOT_DIR, f"H_{int(args.height)}_P_{abs(int(args.pitch))}/{args.weather}"), exist_ok=True)
        os.makedirs(os.path.join(self.ROOT_DIR, f"H_{int(args.height)}_P_{abs(int(args.pitch))}/{args.weather}/{self.town}"))
        os.makedirs(os.path.join(self.ROOT_DIR, f"H_{int(args.height)}_P_{abs(int(args.pitch))}/{args.weather}/{self.town}/Images"))
        os.makedirs(os.path.join(self.ROOT_DIR, f"H_{int(args.height)}_P_{abs(int(args.pitch))}/{args.weather}/{self.town}/CarlaSegment"))
        os.makedirs(os.path.join(self.ROOT_DIR, f"H_{int(self.height)}_P_{abs(int(self.pitch))}/{self.weather_str}/{self.town}/Depth"), exist_ok=True)
        os.makedirs(os.path.join(self.ROOT_DIR, f"H_{int(self.height)}_P_{abs(int(self.pitch))}/{self.weather_str}/{self.town}/Instance"), exist_ok=True)
        ## Save metaData for everything
        os.makedirs(os.path.join(self.ROOT_DIR, f"H_{int(args.height)}_P_{abs(int(args.pitch))}/{self.weather_str}/{self.town}/metaData"))
        
        ### Loading the world
        self.client = carla.Client('localhost', self.port)
        self.client.set_timeout(11.0)
        
        ### Town information
        self.world = self.client.load_world(args.town)
        self.world.wait_for_tick()
        self.settings = self.world.get_settings()
        self.settings.fixed_delta_seconds = 0.05
        self.settings.synchronous_mode = True
        self.world.apply_settings(self.settings)
        self.world.set_weather(self.weather)
        self.actor_list = []

        ### Vehicle to attach all the sensors to
        self.blueprint_library = self.world.get_blueprint_library()
        bp = self.blueprint_library.filter('crossbike')[0] # crossbike is used because it does not cast a shadow
        transform = random.choice(self.world.get_map().get_spawn_points()) # 200 spawn points are available
        self.m = self.world.get_map()
        self.waypoint = self.m.get_waypoint(transform.location)
        self.roadId = self.waypoint.road_id
        self.vehicle = self.world.spawn_actor(bp, transform)
        ## we spawn the vehicle in the air, hence z=self.heightCamera. pitch and roll will always be set to zero
        vehicle_transform = carla.Transform(carla.Location(x=self.waypoint.transform.location.x, y=self.waypoint.transform.location.y, z=self.heightCamera), 
                                             carla.Rotation(pitch=0, yaw=self.waypoint.transform.rotation.yaw, roll=0))
        self.vehicle.set_transform(vehicle_transform)
        self.actor_list.append(self.vehicle)
        self.vehicle.set_autopilot(True)
        self.vehicle.set_enable_gravity(False) # disables gravity
        
        ### Traffic Manager
        self.tm = self.client.get_trafficmanager()
        ## Set up the TM in synchronous mode
        self.tm.set_synchronous_mode(True)
        ## Set a seed so behaviour can be repeated if necessary
        self.tm.set_random_device_seed(0)
        
        self.startTime = time.time()
        self.spawnVehicles()
        self.humansSidewalk()
        self.world.tick()
        print("__"*20)
        self.tickClock()
        print(f'\nTotal Time taken to generate images:  {self.endTime - self.startTime}')
        print(f'Time taken per image:                 {(self.endTime - self.startTime)/self.totalImages}')
    
    def humanManual(self):
        '''
        Spawns humans manually near the driving vehicle to increase the proportion of human class
        '''
        self.peopleSpawned = 0     #counter for number of humans manually spawned in the current iteration
        self.people = []           #list of all the "human" actors spawned in the current iteration, used when destroying the manual actors after taking the snapshot of the scene
        
        prev_wavs = []             #list of waypoints at a distance d behind the vehicle position in the driving lane                            
        all_wavs = []              #list of waypoints at a distance d in front of the vehicle position in the driving lane 
        point =  self.waypoint
        
        while point.road_id == self.waypoint.road_id:
            all_wavs.append(point)
            point = random.choice(point.next(2))
            
        if self.roadId == self.waypoint.road_id:
            point =  self.waypoint
            while point.road_id == self.waypoint.road_id:
                prev_wavs.append(point)
                point = random.choice(point.previous(2))
        else:
            prev_wavs = []
        
        for point in prev_wavs + all_wavs:
            '''Spawn humans at the given location manually'''
            if np.random.normal(0,1) > 0.01:
                
                bp_vehicle = random.choice(self.blueprint_library.filter('walker.*'))
                
                player  = self.world.try_spawn_actor(bp_vehicle, point.transform)
                if player is not None:
                    player_control = carla.WalkerControl()
                    player_control.speed = 3
                    pedestrian_heading=90
                    player_rotation = carla.Rotation(0,pedestrian_heading,0)
                    player_control.direction = player_rotation.get_forward_vector()
                    player.apply_control(player_control)
                    self.people.append(player)
                    self.peopleSpawned += 1

        #randomly pick a waypoint to move the spawning process to next road
        nextRoad = random.choice(point.next(1.5))
        
        #-----------LEFT AND RIGHT LANES SPAWNING-----------------      
        rotation = self.waypoint.transform.rotation

        leftLane = self.waypoint.get_left_lane()                                      #left lane of the driving lane
        rightLane = self.waypoint.get_right_lane()                                    #right lane of the driving lane 
        
        lanesL = []                                                                   #list of all the left lanes of the driving lane                                                     
        lanesR = []                                                                   #list of all the right lanes of the driving lane
        if leftLane !=  None:
            lanesL.append(leftLane)
        if rightLane !=  None:
            lanesR.append(rightLane)
                
        
        while len(lanesL) != 0 or len(lanesR) !=0:
                        
            if len(lanesR) != 0:
                laneR = lanesR.pop(0)
                rightLane = laneR.get_right_lane()

                if rightLane !=  None:
                    rightLane.transform.rotation = rotation
                    lanesR.append(rightLane)
                
            else:
                laneR = None
            
            if len(lanesL) != 0: 
                laneL = lanesL.pop(0)
                if laneL.transform.rotation == rotation:
                    leftLane = laneL.get_left_lane()
                else:
                    leftLane = laneL.get_right_lane()
                if leftLane !=  None:
                        lanesL.append(leftLane)
                    
            else:
                laneL = None
            
            for lane in [laneL, laneR]:
                '''
                Iterate over all the left and right lanes and spawn humans manually 
                '''
                if lane != None:
                    all_wavs = []
                    point = lane
                    
                    while point.road_id == lane.road_id:
                        '''
                        We are only spawning humans in front of the vehicle in the left and right lanes, but the previous points can also be included
                        '''
                        all_wavs.append(point)
                        next = point.next(2)
                        if next != []:
                            point = random.choice(next)
                        else:
                            break
                                                                
                    for point in all_wavs:
                        if np.random.normal(0,1) > 0.01:   
                            bp_vehicle = random.choice(self.blueprint_library.filter('walker.*'))
                            player  = self.world.try_spawn_actor(bp_vehicle, point.transform)
                            if player is not None:
                                player_control = carla.WalkerControl()
                                player_control.speed = 3
                                pedestrian_heading=90
                                player_rotation = carla.Rotation(0,pedestrian_heading,0)
                                player_control.direction = player_rotation.get_forward_vector()
                                player.apply_control(player_control)
                                self.people.append(player)
                                self.peopleSpawned += 1
        
        '''
        Move to the next road, collect waypoints at distance d and repeat the process
        '''
        prevRoadId = nextRoad.road_id
        while self.peopleSpawned < 100: 
            '''The threshold here for number of people to spawn manually varies based on factors like: Town (urban, rural), height of the camera'''
            #--------------JUNCTION OR NEXT ROAD SPAWNING-------------
            if nextRoad.road_id != prevRoadId:
                nextRoad = random.choice(nextRoad.next(5))
                prevRoadId = nextRoad.road_id
                
            all_wavs = []
            point =  nextRoad
            while point.road_id == nextRoad.road_id:
                all_wavs.append(point)
                point = random.choice(point.next(2))
            lanes = []
            
            for point in all_wavs:
                leftLane = point.get_left_lane()
                rightLane = point.get_right_lane()

                if leftLane is not None:
                    lanes.append(leftLane)
                if rightLane is not None:
                    lanes.append(rightLane)
                
                if np.random.normal(0,1) > 0.01:
                    bp_vehicle = random.choice(self.blueprint_library.filter('walker.*'))
                    
                    player  = self.world.try_spawn_actor(bp_vehicle, point.transform)
                    if player is not None:
                        player_control = carla.WalkerControl()
                        player_control.speed = 3
                        pedestrian_heading=90
                        player_rotation = carla.Rotation(0,pedestrian_heading,0)
                        player.apply_control(player_control)
                        self.people.append(player)
                        self.peopleSpawned += 1
            newRoad = random.choice(point.next(1.5))
            
            if len(lanes) == 0:
                leftLane = nextRoad.get_left_lane()
                rightLane = nextRoad.get_right_lane()
                rotation = self.waypoint.transform.rotation
                
                lanesL = []
                lanesR = []
                if leftLane !=  None:
                    lanesL.append(leftLane)
                if rightLane !=  None:
                    lanesR.append(rightLane)
                        
                
                while len(lanesL) != 0 or len(lanesR) !=0:
                    laneL = lanesL.pop(0)
                    laneR = lanesR.pop(0)
                    
                    for lane in [leftLane, rightLane]:
                        if lane != None:
                            all_wavs = random.choice(lane.next(2))
                            all_wavs = lane.next_until_lane_end(2)
                            
                            for point in [all_wavs]:
                                if np.random.normal(0,1) > 0.01:
                                    bp_vehicle = random.choice(self.blueprint_library.filter('walker.*'))
                                    player  = self.world.try_spawn_actor(bp_vehicle, point.transform)
                                    if player is not None:
                                        player_control = carla.WalkerControl()
                                        player_control.speed = 3
                                        pedestrian_heading=90
                                        player_rotation = carla.Rotation(0,pedestrian_heading,0)
                                        player_control.direction = player_rotation.get_forward_vector()
                                        player.apply_control(player_control)
                                        self.people.append(player)
                                        #print(self.waypoint, transform)
                                        self.peopleSpawned += 1
                
            else:
                for point in lanes:
                    if np.random.normal(0,1) > 0.0001:   
                        bp_vehicle = random.choice(self.blueprint_library.filter('walker.*'))
                    
                        player  = self.world.try_spawn_actor(bp_vehicle, point.transform)
                        if player is not None:
                            player_control = carla.WalkerControl()
                            player_control.speed = 3
                            pedestrian_heading=90
                            player_rotation = carla.Rotation(0,pedestrian_heading,0)
                            player.apply_control(player_control)
                            self.people.append(player)
                            #print(self.waypoint, transform)
                            self.peopleSpawned += 1

            nextRoad = newRoad
                
    def humansSidewalk(self):
        """
        Spawn humans randomly on sidewalks(the prescribed way to place humans in the scene)
        """
        humans_start_time = time.time()
        self.peopleSpawnedSidewalk = 0
        self.peopleSidewalk = []
        for _ in range(0, 500):
            transform = carla.Transform()
            transform.location = self.world.get_random_location_from_navigation()
            if transform.location is not None:
                bp_human = random.choice(self.blueprint_library.filter('walker.*'))
                human = self.world.try_spawn_actor(bp_human, transform)
                if human is not None:
                    player_control = carla.WalkerControl()
                    player_control.speed = 3
                    pedestrian_heading = 90
                    player_rotation = carla.Rotation(0, pedestrian_heading, 0)
                    player_control.direction = player_rotation.get_forward_vector()
                    human.apply_control(player_control)
                    self.peopleSidewalk.append(human)
                    self.peopleSpawnedSidewalk += 1
        print(f"Number of peopleSpawnedSidewalk: {self.peopleSpawnedSidewalk} in time: {time.time() - humans_start_time}")         
        
    def spawnVehicles(self):
        '''
         Parameters
         ------
         None
        Returns
        -------
        None.
        '''
        vehicle_start_time = time.time()
        self.vehicles = []
        tm_port = self.tm.get_port()
        for _ in range(0, 500):
            transform = random.choice(self.m.get_spawn_points())
            bp_vehicle = random.choice(self.blueprint_library.filter('vehicle'))
            other_vehicle = self.world.try_spawn_actor(bp_vehicle, transform)
            if other_vehicle is not None:
                other_vehicle.set_autopilot(True, tm_port)
                self.tm.auto_lane_change(other_vehicle, True)
                self.vehicles.append(other_vehicle)
                self.vehiclesSpawned += 1
        print(f"Number of vehicles spawned: {self.vehiclesSpawned} in time: {time.time() - vehicle_start_time}")
    
    def addSensors(self):
        '''
        Paramters:
            self.heightCamera 
            self.pitchCamera
            self.IMG_WIDTH
            self.IMG_HEIGHT

        Function Adds necessary sensors to the scene: GRB camera, segmentation camera, ground RGB camera, Depth camera
        Image size is fixed at IMG_WIDTH X IMG_HEIGHT
        Field of view: 110
        Height of the aerial cameras uses the self.heightCamera parameter
        
        Pitch, roll, yaw: 0
        '''
        ########################################################################################################################
        ####### IMAGES
        ########################################################################################################################
        ##### AERIAL VIEW
        camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('fov', f'{str(self.FOV)}')
        camera_bp.set_attribute('image_size_x', f'{self.IMG_WIDTH}')
        camera_bp.set_attribute('image_size_y', f'{self.IMG_HEIGHT}')
        camera_bp.set_attribute('motion_blur_intensity', '0')
        camera_bp.set_attribute('motion_blur_max_distortion', '0')
        camera_bp.set_attribute('motion_blur_min_object_screen_size', '0')
        camera_bp.set_attribute('blur_amount', '0')
        camera_bp.set_attribute('enable_postprocess_effects', 'True')
        camera_transform = carla.Transform(carla.Location(x=self.SENSOR_X,), # Vehicle is in air, so just x is mentioned
                                           carla.Rotation(pitch=self.pitchCamera, yaw=0, roll=0))
        self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        self.image_queue = queue.Queue()
        self.camera.listen(self.image_queue.put)
        self.actor_list.append(self.camera)

        ########################################################################################################################
        ####### SEMANTIC SEGMENTATION
        ########################################################################################################################
        ##### AERIAL VIEW
        camera_semseg = self.blueprint_library.find('sensor.camera.semantic_segmentation')
        camera_semseg.set_attribute('fov', f'{str(self.FOV)}')
        camera_semseg.set_attribute('image_size_x', f'{self.IMG_WIDTH}')
        camera_semseg.set_attribute('image_size_y', f'{self.IMG_HEIGHT}')
        camera_transform = carla.Transform(carla.Location(x=self.SENSOR_X,), # Vehicle is in air, so just x is mentioned 
                                           carla.Rotation(pitch=self.pitchCamera, yaw=0, roll=0))
        self.camera_seg = self.world.spawn_actor(camera_semseg, camera_transform, attach_to=self.vehicle)
        self.image_queue_seg = queue.Queue()
        self.camera_seg.listen(self.image_queue_seg.put)
        self.actor_list.append(self.camera_seg)
        ########################################################################################################################
        ########################################################################################################################
        ####### DEPTH
        ########################################################################################################################
        ##### AERIAL VIEW
        camera_depth = self.blueprint_library.find('sensor.camera.depth')
        camera_depth.set_attribute('fov', f'{str(self.FOV)}')
        camera_depth.set_attribute('image_size_x', f'{self.IMG_WIDTH}')
        camera_depth.set_attribute('image_size_y', f'{self.IMG_HEIGHT}')
        camera_transform = carla.Transform(carla.Location(x=self.SENSOR_X,), 
                                            carla.Rotation(pitch=self.pitchCamera, yaw=0, roll=0))
        self.camera_depth = self.world.spawn_actor(camera_depth, camera_transform, attach_to=self.vehicle)
        self.image_queue_depth = queue.Queue()
        self.camera_depth.listen(self.image_queue_depth.put)
        self.actor_list.append(self.camera_depth)

        ########################################################################################################################
        ####### INSTANCE 
        ########################################################################################################################
        ##### AERIAL VIEW
        camera_instance = self.blueprint_library.find('sensor.camera.instance_segmentation')
        camera_instance.set_attribute('fov', f'{str(self.FOV)}')
        camera_instance.set_attribute('image_size_x', f'{self.IMG_WIDTH}')
        camera_instance.set_attribute('image_size_y', f'{self.IMG_HEIGHT}')
        camera_transform = carla.Transform(carla.Location(x=self.SENSOR_X,), 
                                            carla.Rotation(pitch=self.pitchCamera, yaw=0, roll=0))
        self.camera_instance = self.world.spawn_actor(camera_instance, camera_transform, attach_to=self.vehicle)
        self.image_queue_instance = queue.Queue()
        self.camera_instance.listen(self.image_queue_instance.put)
        self.actor_list.append(self.camera_instance)
   
    def tickClock(self):
        '''
        Function call to spawn humans randomly in the scene 
        '''
        self.addSensors()
        self.counter = 0
        i = 0
        from tqdm import tqdm 
        pbar = tqdm(total=self.totalImages)
        while(self.counter < self.totalImages):
            if (i%10 == 0):    
                # spawn humans manually only when we are taking a snapshot of the scene
                self.humanManual()
                self.world.tick()
                time.sleep(5)
            else:
                self.world.tick()
            ########################################################################################################################
            ####### IMAGES
            ########################################################################################################################
            ##### AERIAL VIEW
            image = self.image_queue.get()
            ########################################################################################################################
            ####### SEMANTIC SEGMENTATION
            ########################################################################################################################
            ##### AERIAL VIEW
            image_segCarla  = self.image_queue_seg.get()
            image_segCarla.convert(carla.ColorConverter.CityScapesPalette)
            image_depth = self.image_queue_depth.get()
            image_instance = self.image_queue_instance.get()
            ########################################################################################################################
            if i%10 == 0:
                ## Save the image, carla Dict segmented map, depth image, ground scene and carla dict segmented map for ground map
                ########################################################################################################################
                ####### IMAGES
                ########################################################################################################################
                ##### AERIAL VIEW
                IMG_PATH = os.path.join(self.ROOT_DIR, f"H_{int(self.height)}_P_{abs(int(self.pitch))}/{self.weather_str}/{self.town}/Images/{image.frame:06}.png")
                image.save_to_disk(IMG_PATH)
                ########################################################################################################################
                ####### SEMANTIC SEGMENTATION
                ########################################################################################################################
                ##### AERIAL VIEW
                image_segCarla.save_to_disk(os.path.join(self.ROOT_DIR, f"H_{int(self.height)}_P_{abs(int(self.pitch))}/{self.weather_str}/{self.town}/CarlaSegment/{image.frame:06}_semsegCarla.png"))
                image_depth.save_to_disk(os.path.join(self.ROOT_DIR, f"H_{int(self.height)}_P_{abs(int(self.pitch))}/{self.weather_str}/{self.town}/Depth/{image.frame:06}_depth.png"), carla.ColorConverter.LogarithmicDepth)
                # image_depth.save_to_disk(os.path.join(self.ROOT_DIR, f"H_{int(self.height)}_P_{abs(int(self.pitch))}/{self.weather_str}/{self.town}/Depth/{image.frame:06}_depth.png"), carla.ColorConverter.Depth)
                # image_depth.save_to_disk(os.path.join(self.ROOT_DIR, f"H_{int(self.height)}_P_{abs(int(self.pitch))}/{self.weather_str}/{self.town}/Depth/{image.frame:06}_depth.png"))
                image_instance.save_to_disk(os.path.join(self.ROOT_DIR, f"H_{int(self.height)}_P_{abs(int(self.pitch))}/{self.weather_str}/{self.town}/Instance/{image.frame:06}_instance.png"))
                ########################################################################################################################
                data = {}
                data["image_path"] = IMG_PATH
                data["generated"] = True
                data["re-generated"] = False
                data["town"] = self.town
                data["weather"] = self.weather_str
                data["IMG_HEIGHT"] = self.IMG_HEIGHT
                data["IMG_WIDTH"] = self.IMG_WIDTH
                data["SIGMA_H"] = self.SIGMA_H
                data["SIGMA_P"] = self.SIGMA_P
                data["height"] = self.height
                data["pitch"] = self.pitch
                data["actual_height"] = self.heightCamera
                data["actual_pitch"] = self.pitchCamera
                data["ego_vehicle"] = str(self.vehicle.get_transform())
                data["num_walkers_spawned"] = self.peopleSpawned
                data["num_walkers_spawned_sidewalk"] = self.peopleSpawnedSidewalk
                data["total_num_walkers"] = self.peopleSpawned + self.peopleSpawnedSidewalk
                data["total_num_vehicles"] = self.vehiclesSpawned
                data["actual_num_walkers"] = data["total_num_walkers"]
                data["actual_num_vehicles"] = data["total_num_vehicles"]
                
                data["sensors"] = [str(item) + "\n" + str(item.get_transform()) + "\n" for item in self.actor_list]
                data["walkers_spawned_sidewalk"] = [str(item) + "\n" + str(item.get_transform()) + "\n" for item in self.peopleSidewalk]
                data["walkers_spawned"] = [str(item) + "\n" + str(item.get_transform()) + "\n" for item in self.people]
                
                data["vehicles"] = [str(item) + "\n" + str(item.get_transform()) + "\n" for item in self.vehicles]
                data["walkers"] = data["walkers_spawned_sidewalk"] + data["walkers_spawned"]
                
                JSON_PATH = os.path.join(self.ROOT_DIR, f"H_{int(self.height)}_P_{abs(int(self.pitch))}/{self.weather_str}/{self.town}/metaData/{image.frame:06}.json")
                with open(JSON_PATH, "w") as json_file:
                    json.dump(data, json_file)

                self.counter += 1
                pbar.update(1)
                self.destroypeople() # manual humans destroyed, that are randomly added to scenes

            self.waypoint = random.choice(self.waypoint.next(1.5))
            self.heightCamera = np.random.normal(int(self.height), self.SIGMA_H)
            vehicle_transform = carla.Transform(carla.Location(x=self.waypoint.transform.location.x, y=self.waypoint.transform.location.y, z=self.heightCamera),
                                             carla.Rotation(pitch=0, yaw=self.waypoint.transform.rotation.yaw, roll=0))
            self.vehicle.set_transform(vehicle_transform)
            
            self.pitchCamera = np.random.normal(int(self.pitch), self.SIGMA_P)
            cam_transform = carla.Transform(carla.Location(x=self.SENSOR_X,), # Vehicle is in air, so just x is mentioned 
                                           carla.Rotation(pitch=self.pitchCamera, yaw=0, roll=0))
            self.camera.set_transform(cam_transform)
            self.camera_seg.set_transform(cam_transform)
            
            i += 1
            self.roadId = self.waypoint.road_id
        pbar.close()
        self.destroypeople()
        self.destroyActors()
          
    def destroyActors(self):
        self.camera.destroy()
        self.camera_seg.destroy()
        self.camera_depth.destroy()
        self.camera_instance.destroy()
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles])
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.peopleSidewalk])
        self.endTime = time.time()   

    def destroypeople(self):
        """
        Destroys manually added people
        This function is called after every saved image
        """
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.people]) 
        self.people = []


if __name__ == "__main__":
    ########################################################################################################################
    ### CMD LINE ARGUMENTS
    ########################################################################################################################
    parser = argparse.ArgumentParser()
    parser.add_argument('--weather', type=str, default="ClearNoon", help="ClearNoon CloudyNoon MidRainyNoon ClearSunset ClearNight")
    parser.add_argument('--town', type=str, default="Town01", help="Town01 Town02 Town03 Town04 Town05 Town06 Town07 Town10HD")
    parser.add_argument('--ROOT_DIR', type=str, default="temp", help="Dir to save")
    parser.add_argument('--height', type=int, default=35, help="height")
    parser.add_argument('--pitch', type=int, default=-45, help="pitch")
    parser.add_argument('--num', type=int, default=10, help="number of images to generate")
    args = parser.parse_args()
    ########################################################################################################################
    ### MANUAL ARGUMENTS
    ########################################################################################################################
    args.ROOT_DIR = f"INIT_DATA"
    args.town = "Town01"       # Town01 Town02 Town03 Town04 Town05 Town06 Town07 Town10HD
    args.weather = "ClearNoon" # ClearNoon CloudyNoon MidRainyNoon ClearSunset ClearNight
    args.height = 35
    args.pitch = -45
    args.num = 5

    print("__"*20)
    print(f"The arguments for generation are as follows: ")
    print(f"Town:           {args.town}")
    print(f"Weather:        {args.weather}")
    print(f"Height:         {args.height}")
    print(f"Pitch:          {args.pitch}")
    print(f"Num of images:  {args.num}")
    gn = genImages(args)
