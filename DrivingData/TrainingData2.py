import glob
import os
import sys
import time
import math
import weakref
import pandas as pd
import numpy as np
from tqdm import tqdm

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import random
import dynamic_weather as dweather

save_path = "C:/CARLA_0.9.10/WindowsNoEditor/PythonAPI/examples/Saved/record_9/"
replay_time = 10000

if True:
    load_world = "Town06"
    #load_world = "Town10HD"
else:
    load_world = None

def main():
    start_time = time.time()
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    vehicles_list = []
    number_of_vehicles = 0
    tm_port = 8000
    img_size_x = 400
    img_size_y = 300
    wait_time = 0.08
    use_dw = false
    dw_speed = 0.1
    fixed_delta_seconds = 0.1
    global_percentage_speed_difference = 10

    if load_world != None:
        print(client.get_available_maps())
        client.load_world(load_world)
        client.reload_world()

        time.sleep(3)

    #update the dict to contain traffic light information and speed limit!
    # should include wheel steer angle??
    ddict ={'t':[],'x':[],'y':[],'z':[],'pitch':[],'yaw':[],'roll':[],'vx':[], 'vy':[], 'vz':[], 'throttle':[], 'steer':[], 'brake':[], 'reverse':[], 'gear':[]}

    try:

        world = client.get_world() 
        ego_vehicle = None
        ego_cam = None
        depth_cam = None
        depth_cam02 = None
        sem_cam = None
        rad_ego = None
        lidar_sen = None



        # Set Synchronomous mode True
        synchronous_master = True
        traffic_manager = client.get_trafficmanager(tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        traffic_manager.global_percentage_speed_difference(global_percentage_speed_difference) #speeds up all the vehicles to 130% of speed limit

        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = fixed_delta_seconds
        world.apply_settings(settings)

        # --------------
        # Query the recording
        # --------------
        #"""
        # Show the most important events in the recording.  
        #print(client.show_recorder_file_info(save_path + "manual_recording2.rec",False))
        # Show actors not moving 1 meter in 10 seconds.  
        #print(client.show_recorder_actors_blocked("~/tutorial/recorder/recording04.log",10,1))
        # Show collisions between any type of actor.  
        #print(client.show_recorder_collisions("~/tutorial/recorder/recording04.log",'v','a'))
        #"""

        # --------------
        # Reenact a fragment of the recording
        # --------------


        # --------------
        # Set playback simulation conditions
        # --------------
        #"""
        blueprints = world.get_blueprint_library().filter('vehicle.*')
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        SpawnActor = carla.command.SpawnActor
        SetAutoPilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        if number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        else:
            print(f"Requested {number_of_vehicles} vehicles, but could only find {number_of_spawn_points} spawn points")
            number_of_vehicles = number_of_spawn_points - 1
            random.shuffle(spawn_points)

        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(blueprint, transform).then(SetAutoPilot(FutureActor,True)))
            spawn_points.pop(0)

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        print(f"Created {len(vehicles_list)} npc vehicles")

        #ego_bp = random.choice(blueprints)
        ego_bp = blueprints[0]
        ego_transform = spawn_points[-1]
        ego_vehicle = world.spawn_actor(ego_bp, ego_transform)
        ego_vehicle.set_autopilot(True)

        print("Ego-vehicle ready")
        #"""
        # --------------
        # Place spectator on ego spawning
        # --------------
        #"""
        spectator = world.get_spectator()
        spectator.set_transform(ego_vehicle.get_transform())
        #"""

        # --------------
        # Change weather conditions
        # --------------
        """
        weather = world.get_weather()
        weather.sun_altitude_angle = -30
        weather.fog_density = 65
        weather.fog_distance = 10
        world.set_weather(weather)
        """
        # dynamic_weather
        if use_dw:
            update_freq = 0.1/dw_speed
            weather = dweather.Weather(world.get_weather())
            elapsed_time = 0
            weather.tick(dw_speed)
            world.set_weather(weather.weather) 
        # --------------
        # Add a RGB camera to ego vehicle.
        # --------------
        #"""
        cam_bp = None
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_location = carla.Location(2,0,1)
        cam_rotation = carla.Rotation(0,0,0)
        cam_transform = carla.Transform(cam_location,cam_rotation)
        cam_bp.set_attribute("image_size_x",str(img_size_x))
        cam_bp.set_attribute("image_size_y",str(img_size_y))
        cam_bp.set_attribute("fov",str(105))
        ego_cam = world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        ego_cam.listen(lambda image: image.save_to_disk( save_path + 'new_rgb_output/%.6d.jpg' % image.frame))
        #"""

        # --------------
        # Add a Logarithmic Depth camera to ego vehicle. 
        # --------------
        #"""
        depth_cam = None
        depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
        depth_bp.set_attribute("image_size_x",str(img_size_x))
        depth_bp.set_attribute("image_size_y",str(img_size_y))
        depth_bp.set_attribute("fov",str(105))
        depth_location = carla.Location(2,0,1)
        depth_rotation = carla.Rotation(0,0,0)
        depth_transform = carla.Transform(depth_location,depth_rotation)
        depth_cam = world.spawn_actor(depth_bp,depth_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # This time, a color converter is applied to the image, to get the semantic segmentation view
        #depth_cam.listen(lambda image: image.save_to_disk(save_path + 'de_log/%.6d.jpg' % image.frame,carla.ColorConverter.LogarithmicDepth))
        #"""
        # --------------
        # Add a Depth camera to ego vehicle. 
        # --------------
        #"""
        depth_cam02 = None
        depth_bp02 = world.get_blueprint_library().find('sensor.camera.depth')
        depth_bp02.set_attribute("image_size_x",str(img_size_x))
        depth_bp02.set_attribute("image_size_y",str(img_size_y))
        depth_bp02.set_attribute("fov",str(105))
        depth_location02 = carla.Location(2,0,1)
        depth_rotation02 = carla.Rotation(0,0,0)
        depth_transform02 = carla.Transform(depth_location02,depth_rotation02)
        depth_cam02 = world.spawn_actor(depth_bp02,depth_transform02,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # This time, a color converter is applied to the image, to get the semantic segmentation view
        depth_cam02.listen(lambda image: image.save_to_disk(save_path + 'de/%.6d.jpg' % image.frame,carla.ColorConverter.Depth))
        #"""

        # --------------
        # Add a new semantic segmentation camera to ego vehicle
        # --------------
        #"""
        sem_cam = None
        sem_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        sem_bp.set_attribute("image_size_x",str(img_size_x))
        sem_bp.set_attribute("image_size_y",str(img_size_y))
        sem_bp.set_attribute("fov",str(105))
        sem_location = carla.Location(2,0,1)
        sem_rotation = carla.Rotation(0,0,0)
        sem_transform = carla.Transform(sem_location,sem_rotation)
        sem_cam = world.spawn_actor(sem_bp,sem_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # This time, a color converter is applied to the image, to get the semantic segmentation view
        sem_cam.listen(lambda image: image.save_to_disk(save_path + 'new_sem_output/%.6d.jpg' % image.frame,carla.ColorConverter.CityScapesPalette))
        #"""

        # --------------
        # Add a new radar sensor to ego vehicle
        # --------------
        #"""

        # --------------
        # Add a new LIDAR sensor to ego vehicle
        # --------------
        #"""
        lidar_cam = None
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels',str(32))
        lidar_bp.set_attribute('points_per_second',str(150000))
        lidar_bp.set_attribute('rotation_frequency',str(40))
        lidar_bp.set_attribute('range',str(50))
        lidar_location = carla.Location(0,0,2)
        lidar_rotation = carla.Rotation(0,0,0)
        lidar_transform = carla.Transform(lidar_location,lidar_rotation)
        lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle,attachment_type=carla.AttachmentType.Rigid)
        lidar_sen.listen(lambda point_cloud: point_cloud.save_to_disk(save_path+'new_lidar_output/%.6d.ply' % point_cloud.frame))
        #"""

        

        # --------------
        # Game loop. Prevents the script from finishing.
        # --------------
        for i in tqdm(range(replay_time)):
            world_snapshot = world.tick()
            ddict['t'].append(i*fixed_delta_seconds)
            ev = ego_vehicle.get_velocity()
            
            vx = ev.x
            vy = ev.y
            vz = ev.z

            ec = ego_vehicle.get_control()
            throttle = ec.throttle
            steer = ec.steer
            brake = ec.brake
            reverse = ec.reverse
            gear = ec.gear

            et = ego_vehicle.get_transform()

            el = et.location
            er = et.rotation

            x = el.x
            y = el.y
            z = el.z

            pitch = er.pitch
            yaw = er.yaw
            roll = er.roll

            ddict['x'].append(x)
            ddict['y'].append(y)
            ddict['z'].append(z)

            ddict['pitch'].append(pitch)
            ddict['roll'].append(roll)
            ddict['yaw'].append(yaw)

            ddict['vx'].append(vx)
            ddict['vy'].append(vy)
            ddict['vz'].append(vz)

            ddict['throttle'].append(throttle)
            ddict['steer'].append(steer)
            ddict['brake'].append(brake)
            ddict['reverse'].append(reverse)
            ddict['gear'].append(gear)
            #sys.stdout.write(str(ego_vehicle.get_velocity()) + '\n')

            if use_dw:
                elapsed_time += fixed_delta_seconds
                if elapsed_time > update_freq:
                    weather.tick(dw_speed*elapsed_time)
                    world.set_weather(weather.weather)
                    elapsed_time = 0
            time.sleep(wait_time)

        print('Breaking out of sim. (Time Out)')
        df = pd.DataFrame(ddict)
        df.to_csv(f"{save_path}vlog.csv", index=False)
        # --------------
        # Destroy actors
        # --------------
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        if ego_vehicle is not None:
            if ego_cam is not None:
                ego_cam.stop()
                ego_cam.destroy()
            if depth_cam is not None:
                depth_cam.stop()
                depth_cam.destroy()
            if sem_cam is not None:
                sem_cam.stop()
                sem_cam.destroy()
            if rad_ego is not None:
                rad_ego.stop()
                rad_ego.destroy()
            if lidar_sen is not None:
                lidar_sen.stop()
                lidar_sen.destroy()
           
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        ego_vehicle.destroy()
        
        print('\nNothing to be done.')

    except Exception as e:
        print(e)
    finally:
        # --------------
        # Destroy actors
        # --------------
        if ego_vehicle is not None:
            if ego_cam is not None:
                ego_cam.stop()
                ego_cam.destroy()
            if depth_cam is not None:
                depth_cam.stop()
                depth_cam.destroy()
            if sem_cam is not None:
                sem_cam.stop()
                sem_cam.destroy()
            if rad_ego is not None:
                rad_ego.stop()
                rad_ego.destroy()
            if lidar_sen is not None:
                lidar_sen.stop()
                lidar_sen.destroy()
            ego_vehicle.destroy()

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        print('\nNothing to be done.')


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with tutorial_replay.')