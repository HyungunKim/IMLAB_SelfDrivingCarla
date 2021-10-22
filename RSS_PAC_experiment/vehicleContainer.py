# m = world.world.get_map()
# w = m.get_waypoint(world.vehicle.get_location())
# next_until_lane_end(d)
# get_right_lane()


class vehicle_control:
    def __init__(self, ego_vehicle):
        self.ddict ={'t':[],'x':[],'y':[],'z':[],'pitch':[],'yaw':[],'roll':[],
                'vx':[], 'vy':[], 'vz':[], 'throttle':[], 'steer':[], 'brake':[], 
                'reverse':[], 'gear':[]}

        self.ego_vehicle = ego_vehicle


    def record(timestep):
        self.ddict['t'].append(timestep * fixed_delta_seconds)

        ev = self.ego_vehicle.get_velocity()
            
        vx = ev.x
        vy = ev.y
        vz = ev.z

        ec = self.ego_vehicle.get_control()
        throttle = ec.throttle
        steer = ec.steer
        brake = ec.brake
        reverse = ec.reverse
        gear = ec.gear

        et = self.ego_vehicle.get_transform()

        el = et.location
        er = et.rotation

        x = el.x
        y = el.y
        z = el.z

        pitch = er.pitch
        yaw = er.yaw
        roll = er.roll

        self.ddict['x'].append(x)
        self.ddict['y'].append(y)
        self.ddict['z'].append(z)

        self.ddict['pitch'].append(pitch)
        self.ddict['roll'].append(roll)
        self.ddict['yaw'].append(yaw)

        self.ddict['vx'].append(vx)
        self.ddict['vy'].append(vy)
        self.ddict['vz'].append(vz)

        self.ddict['throttle'].append(throttle)
        self.ddict['steer'].append(steer)
        self.ddict['brake'].append(brake)
        self.ddict['reverse'].append(reverse)
        self.ddict['gear'].append(gear)self

class vehicleContainer:
    def __init__(self, vehicle, client, sensors=[]):
        self.vehicle = vehicle
        self.client = client
        self.vehicle_control = vehicle_control
        self.sensors = sensors

    def get_waypoint(self, map):


