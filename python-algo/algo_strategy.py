import gamelib
import random
import math
import warnings
from sys import maxsize
import json
from collections import defaultdict
import numpy as np

"""
Most of the algo code you write will be in this file unless you create new
modules yourself. Start by modifying the 'on_turn' function.

Advanced strategy tips: 

  - You can analyze action frames by modifying on_action_frame function

  - The GameState.map object can be manually manipulated to create hypothetical 
  board states. Though, we recommended making a copy of the map to preserve 
  the actual current map state.
"""

class AlgoStrategy(gamelib.AlgoCore):
    def __init__(self):
        super().__init__()
        seed = random.randrange(maxsize)
        random.seed(seed)
        gamelib.debug_write('Random seed: {}'.format(seed))

    def on_game_start(self, config):
        """ 
        Read in config and perform any initial setup here 
        """
        gamelib.debug_write('Configuring your custom algo strategy...')
        self.config = config
        global WALL, SUPPORT, TURRET, SCOUT, DEMOLISHER, INTERCEPTOR, MP, SP, WIDTH
        WALL = config["unitInformation"][0]["shorthand"]
        SUPPORT = config["unitInformation"][1]["shorthand"]
        TURRET = config["unitInformation"][2]["shorthand"]
        SCOUT = config["unitInformation"][3]["shorthand"]
        DEMOLISHER = config["unitInformation"][4]["shorthand"]
        INTERCEPTOR = config["unitInformation"][5]["shorthand"]
        MP = 1
        SP = 0
        WIDTH = 28

        # unit's health
        self.units_health = {
            WALL: config["unitInformation"][0]["startHealth"],
            SUPPORT: config["unitInformation"][1]["startHealth"],
            TURRET: config["unitInformation"][2]["startHealth"],
            SCOUT: config["unitInformation"][3]["startHealth"],
            DEMOLISHER: config["unitInformation"][4]["startHealth"],
            INTERCEPTOR: config["unitInformation"][5]["startHealth"]
        }
        # unit's cost
        self.units_cost = {
            WALL: config["unitInformation"][0]["cost"],
            SUPPORT: config["unitInformation"][1]["cost"],
            TURRET: config["unitInformation"][2]["cost"],
            SCOUT: config["unitInformation"][3]["cost"],
            DEMOLISHER: config["unitInformation"][4]["cost"],
            INTERCEPTOR: config["unitInformation"][5]["cost"]
        }


        # stuff for enemy analysis
        self.first_frame_captured = False
        self.enemy_structure_prev = [[0] * WIDTH] * WIDTH
        self.enemy_structure_heatmap = {
            WALL: [[0] * WIDTH] * WIDTH,
            SUPPORT: [[0] * WIDTH] * WIDTH,
            TURRET: [[0] * WIDTH] * WIDTH
        }
        self.enemy_deploy_heatmap = {
            SCOUT: [0] * WIDTH,
            DEMOLISHER: [0] * WIDTH,
            INTERCEPTOR: [0] * WIDTH
        }
        self.enemy_deploy_num = {
            SCOUT: [[]] * WIDTH,
            DEMOLISHER: [[]] * WIDTH,
            INTERCEPTOR: [[]] * WIDTH
        }

        ##Defenders
        ##key: unit's shorthand, value: [locations, damage taken]
        self.breached_defenders = defaultdict(list) 
        # self.removed_defenders = defaultdict(list)

    def on_turn(self, turn_state):
        """
        This function is called every turn with the game state wrapper as
        an argument. The wrapper stores the state of the arena and has methods
        for querying its state, allocating your current resources as planned
        unit deployments, and transmitting your intended deployments to the
        game engine.
        """
        game_state = gamelib.GameState(self.config, turn_state)
        gamelib.debug_write('Performing turn {} of your custom algo strategy'.format(game_state.turn_number))
        game_state.suppress_warnings(True)  #Comment or remove this line to enable warnings.
        ## place defense only on 1st turn
        self.init_defense(game_state)
        self.defense_strategy(game_state)
        ## self.starter_strategy(game_state)

        game_state.submit_turn()

        # reset enemy analysis flag
        self.first_frame_captured = False

    def on_action_frame(self, turn_string):
        """
        This is the action frame of the game. This function could be called 
        hundreds of times per turn and could slow the algo down so avoid putting slow code here.
        Processing the action frames is complicated so we only suggest it if you have time and experience.
        Full doc on format of a game frame at in json-docs.html in the root of the Starterkit.
        """
        # read in the state
        state = json.loads(turn_string)
        # analyze enemy moves
        if not self.first_frame_captured:
            self.first_frame_captured = True
            self.analyze_enemy(state)
        # Let's record at what position we get scored on
        events = state["events"]
        breaches = events["breach"]
        damages = events["damages"]
        defenders = {2:TURRET,0:WALL}
        self.breached_defenders = defaultdict(list)
        for breach in breaches:
            location = breach[0]
            unit_owner_self = True if breach[4] == 1 else False
            unit_type = breach[2]
            if unit_owner_self:
                self.breached_defenders[defenders[unit_type]].append(location)
            # When parsing the frame data directly, 
            # 1 is integer for yourself, 2 is opponent (StarterKit code uses 0, 1 as player_index instead)
            if not unit_owner_self:
                gamelib.debug_write("Got scored on at: {}".format(location))
                self.scored_on_locations.append(location)
                gamelib.debug_write("All locations: {}".format(self.scored_on_locations))

        ## get all damaged defenders
        # self.damaged_defenders = {}
        # defenders = {2:TURRET, 0:WALL}
        # for damage in damages:
        #     location = damage[0]
        #     unit_owner_self = True if damage[4] == 1 else False
        #     unit = damage[2]
        #     damage_taken = damage[1]
        #     if unit_owner_self and unit in defenders:
        #         self.damaged_defenders[defenders[unit]].append((location, damage_taken))

    """
    ------------------------------------------Our code below:
    """
    def analyze_enemy(self, frame_state):
        # A helper to track enemy moves for enemy analysis
        enemy_structure_current = [[0] * WIDTH] * WIDTH
        for index in range(6):
            unit_type = [WALL, SUPPORT, TURRET, SCOUT, DEMOLISHER, INTERCEPTOR][index]
            unit_num = {}
            for unit in frame_state['p2Units'][index]:
                x_coord, y_coord = unit[0], unit[1]
                if unit_type in [WALL, SUPPORT, TURRET]:
                    enemy_structure_current[x_coord][y_coord] = 1
                    if self.enemy_structure_prev[x_coord][y_coord] == 0:
                        self.enemy_structure_heatmap[unit_type][x_coord][y_coord] += 0.5                # TODO: TUNABLE
                    else:
                        self.enemy_structure_heatmap[unit_type][x_coord][y_coord] += 1
                else:
                    if x_coord in unit_num:
                        unit_num[x_coord] += 1
                    else:
                        unit_num[x_coord] = 1
            if unit_type in self.enemy_deploy_heatmap:
                for x_coord, num in unit_num:
                    self.enemy_deploy_heatmap[unit_type][x_coord] += num
                    self.enemy_deploy_num[unit_type][x_coord].append(num)
        self.enemy_structure_prev = enemy_structure_current

    def predict_enemy_structure(self, game_state, possibility_bar = 0.8):
        # Returns predicted_enemy_map
        enemy_sp = game_state.get_resource(0, 1)
        norm_enemy_structure_heatmap = []
        # normalize and convert heatmap to np array
        for unit_type in [WALL, SUPPORT, TURRET]:
            norm_enemy_structure_heatmap.append(np.array(self.enemy_structure_heatmap[unit_type]) / game_state.turn_number) # list(map(lambda x: list(map(lambda: y: y / game_state.turn_number, x)), self.enemy_structure_heatmap[unit_type]))            # NOTE: very likely to have bugs
        norm_enemy_structure_heatmap = np.array(norm_enemy_structure_heatmap)    # (3x28x28)
        # filter out likely locations and sort by likelihood
        norm_enemy_structure_predict = np.argwhere(norm_enemy_structure_heatmap > possibility_bar)      # TODO: TUNABLE
        order = np.argsort(norm_enemy_structure_heatmap[norm_enemy_structure_predict])                  # need to check if this map access is correct
        norm_enemy_structure_predict = norm_enemy_structure_predict[order]
        # add likely positions to the map
        for index, x_coord, y_coord in norm_enemy_structure_predict:
            if len(game_state[x_coord][y_coord]) > 0:
                continue
            enemy_sp -= self.units_cost[unit_type]
            if enemy_sp < 0:
                break
            predicted_game_state.game_map.add_unit([WALL, SUPPORT, TURRET][index], [x,y], 1)
        return predicted_game_state

    def predict_enemy_attack(self, game_state):
        # Returns possible breach location
        enemy_deploy_tensor = []
        enemy_deploy_num = []
        for unit_type in [WALL, SUPPORT, TURRET]:
            enemy_deploy_tensor.append(self.enemy_deploy_heatmap[unit_type])
            enemy_deploy_num.append(self.enemy_deploy_num[unit_type])
        enemy_deploy_tensor = np.linalg.norm(np.array(enemy_deploy_tensor))     # (3x28)
        enemy_deploy_location = np.argwhere(enemy_deploy_tensor > 0)                                    # TODO: TUNABLE
        order = np.argsort(self.enemy_deploy_heatmap[enemy_deploy_location])
        enemy_deploy_location = enemy_deploy_location[order]
        result = []
        for index, x_coord in enemy_deploy_location:
            coords = self.get_edge_location(27 - x_coord)
            num_deployed = enemy_deploy_num[index][x_coord]
            num_mode = max(set(num_deployed), key = num_deployed.count)
            simulation = self.single_attack_path_analysis(game_state, coords, num_mode, [WALL, SUPPORT, TURRET][index])
            breach_pos = simulation[-1]
            result.append(breach_pos, enemy_deploy_tensor[index, x_coord])
        return result
    
    def get_damged_defenders(self, game_state):
        #key: unit type, value: health ratio
        damaged_defenders = defaultdict(list)
        defenders = [TURRET, WALL]
        for location in game_state.game_map:
            unit = game_state.game_map[location]
            unit_owned = unit.player_index
            if unit_owned and unit.unit_type in defenders:
                health_ratio = unit.health / unit.max_health
                damaged_defenders[unit.unit_type].append((location, health_ratio))
        return damaged_defenders
    
    # upgrade defenders
    def upgrade_denfense(self, game_state, damaged_defenders, unit_type, threshold):
        for locaiton, health_ratio in damaged_defenders[unit_type]:
            if health_ratio > threshold:
                game_state.attempt_upgrade(location)
    
    def recycle_defenders(self, game_state, damaged_defenders, unit_type, threshold):
        for locaiton, health_ratio in damaged_defenders[TURRET]:
            if health_ratio > threshold:
                game_state.attempt_remove(location)
        
    def respawn_defenders(self, game_state, breached_defenders, unit_type, threshold):
        temp = sorted(self.breached_defenders[unit_type], key=lambda: x[0])
        for location in temp:
            
            



    ## TODO 
    def defense_strategy(self, game_state): 
        # get all damaged defenders
        damaged_defenders = self.get_damged_defenders(self, game_state)
    
        # priorities:
        # 1. re-spawn breached defenders
        
        # 2. upgrade possible breach corners
        for location in self.predict_enemy_attack:
            game_state.attempt_upgrade(location)
        
        # 3. recycle damaged turrets, whose health in within a threshold
        health_threshold = 0.8
        self.recycle_defenders(game_state, damaged_defenders, TURRET, health_threshold)
        self.upgrade_denfense(game_state, damaged_defenders, TURRET, health_threshold)
        

        # 4. upgrade turrets
        
        
        # 5. recycle damaged turrets, whose health in within a threshold
        for locaiton, health_ratio in damaged_defenders[WALL]:
            if health_ratio > health_threshold:
                game_state.attempt_remove(location)
          

        return shit


    ## RW TODO
    def single_attack_path_analysis(self, game_state, location, num, unittype):
        # For each deployment position, try to investigate the attack path and potential result 
        # based on the current friendly board and analyzed/predicted enemy board.
        # return (can_reach, structure_demolished, damage_dealt, points_gain, first_attach_pos, breach_pos)
        can_reach = 0
        _dem_ = 0
        _dmg_ = 0
        _pnt_ = 0
        # add all normal supports into a set from game_state
        normal_supp_set = set()
        upgrade_supp_set = set()
        for i in range(14):
            for j in range(13-i, 13+i+1):
                if game_state.map[j][i][0].unit_type == SUPPORT:
                    if game_state.map[j][i][0].upgraded == 0:
                        supp_set.add((game_state.map[j][i][0].x, game_state.map[j][i][0].y))
                    else:
                        upgrade_supp_set.add((game_state.map[j][i][0].x, game_state.map[j][i][0].y))

        target_edge = get_target_edge(location)
        path = find_path_to_edge(location, edge)
        ind = 0

        # initialize the total health of the deployment, and accommodate the path by the speed of the unittype (in frame count)
        _hp_ = 0
        adj_path = []
        if unittype == SCOUT:
            _hp_ = num * 15
        elif unittype == DEMOLISHER:
            _hp_ = num * 5
            for cord in path:
                for i in range(2):
                    adj_path.append(cord)
        else:
            _hp_ = num * 40
            for cord in path:
                for i in range(4):
                    adj_path.append(cord)
                    
        # analyze the coordinate one by one in the adjusted path (in frame count)
        while ind < len(path):
            cur_pos = path[ind] # current coordinate

            # Step 1: Add shields through nearby supports
            # normal support
            for nss in normal_supp_set:
                if nss.shieldRange > euclid_dist(nss, cur_pos):
                    _hp_ += 3 * num
                    normal_supp_set.remove(nss)
            # upgraded support
            for uss in upgrade_supp_set:
                if uss.shieldRange > euclid_dist(uss, cur_pos):
                    _hp_ += (4 + 0.3 * uss[1]) * num
                    normal_supp_set.remove(nss)
            
            # Step 2: in each frame, calculate the damage we dealt to 
        
    """
    ----------------------------helper functions----------------------------
    """
    
    def get_edge_location(x):
        # return edge coordinates given x
        return (x, abs(math.floor(13.5-x)))
    
    def euclid_dist(loc1, loc2):
        dist = sqrt(pow(loc1[0] - loc2[0], 2) + pow(loc1[1] - loc2[1], 2))
        return dist

if __name__ == "__main__":
    algo = AlgoStrategy()
    algo.start()



class Old:
        """
    NOTE: All the methods after this point are part of the sample starter-algo
    strategy and can safely be replaced for your custom algo.
    """
    def starter_strategy(self, game_state):
        """
        For defense we will use a spread out layout and some interceptors early on.
        We will place turrets near locations the opponent managed to score on.
        For offense we will use long range demolishers if they place stationary units near the enemy's front.
        If there are no stationary units to attack in the front, we will send Scouts to try and score quickly.
        """
        # First, place basic defenses
        # self.build_defences(game_state)
        # Now build reactive defenses based on where the enemy scored
        self.build_reactive_defense(game_state)

        # If the turn is less than 5, stall with interceptors and wait to see enemy's base
        if game_state.turn_number < 5:
            self.stall_with_interceptors(game_state)
        else:
            # Now let's analyze the enemy base to see where their defenses are concentrated.
            # If they have many units in the front we can build a line for our demolishers to attack them at long range.
            if self.detect_enemy_unit(game_state, unit_type=None, valid_x=None, valid_y=[14, 15]) > 10:
                self.demolisher_line_strategy(game_state)
            else:
                # They don't have many units in the front so lets figure out their least defended area and send Scouts there.

                # Only spawn Scouts every other turn
                # Sending more at once is better since attacks can only hit a single scout at a time
                if game_state.turn_number % 2 == 1:
                    # To simplify we will just check sending them from back left and right
                    scout_spawn_location_options = [[13, 0], [14, 0]]
                    best_location = self.least_damage_spawn_location(game_state, scout_spawn_location_options)
                    game_state.attempt_spawn(SCOUT, best_location, 1000)

                # Lastly, if we have spare SP, let's build some Factories to generate more resources
                support_locations = [[13, 2], [14, 2], [13, 3], [14, 3]]
                game_state.attempt_spawn(SUPPORT, support_locations)

    def init_defences(self, game_state):
        """
        Build basic defenses using hardcoded locations.
        Remember to defend corners and avoid placing units in the front where enemy demolishers can attack them.
        """
        # Useful tool for setting up your base locations: https://www.kevinbai.design/terminal-map-maker
        # More community tools available at: https://terminal.c1games.com/rules#Download


        # Place turrets that attack enemy units
        turret_locations = [[3, 12], [8, 12], [19, 12], [24, 12], [13, 11], [14, 11]]
        # attempt_spawn will try to spawn units if we have resources, and will check if a blocking unit is already there
        game_state.attempt_spawn(TURRET, turret_locations)
        
        # Place walls in front of turrets to soak up damage for them
        wall_locations = [[0, 13], [1, 13], [2, 13], [3, 13], [8, 13], [19, 13], [24, 13], [25, 13], [26, 13], [27, 13], [7, 12], [9, 12], [12, 12], [13, 12], [14, 12], [15, 12], [18, 12], [20, 12]]
        game_state.attempt_spawn(WALL, wall_locations)

    def build_reactive_defense(self, game_state):
        """
        This function builds reactive defenses based on where the enemy scored on us from.
        We can track where the opponent scored by looking at events in action frames 
        as shown in the on_action_frame function
        """
        
        for location in self.scored_on_locations:
            # Build turret one space above so that it doesn't block our own edge spawn locations
            build_location = [location[0], location[1]+1]
            game_state.attempt_spawn(TURRET, build_location)

    def stall_with_interceptors(self, game_state):
        """
        Send out interceptors at random locations to defend our base from enemy moving units.
        """
        # We can spawn moving units on our edges so a list of all our edge locations
        friendly_edges = game_state.game_map.get_edge_locations(game_state.game_map.BOTTOM_LEFT) + game_state.game_map.get_edge_locations(game_state.game_map.BOTTOM_RIGHT)
        
        # Remove locations that are blocked by our own structures 
        # since we can't deploy units there.
        deploy_locations = self.filter_blocked_locations(friendly_edges, game_state)
        
        # While we have remaining MP to spend lets send out interceptors randomly.
        while game_state.get_resource(MP) >= game_state.type_cost(INTERCEPTOR)[MP] and len(deploy_locations) > 0:
            # Choose a random deploy location.
            deploy_index = random.randint(0, len(deploy_locations) - 1)
            deploy_location = deploy_locations[deploy_index]
            
            game_state.attempt_spawn(INTERCEPTOR, deploy_location)
            """
            We don't have to remove the location since multiple mobile 
            units can occupy the same space.
            """

    def demolisher_line_strategy(self, game_state):
        """
        Build a line of the cheapest stationary unit so our demolisher can attack from long range.
        """
        # First let's figure out the cheapest unit
        # We could just check the game rules, but this demonstrates how to use the GameUnit class
        stationary_units = [WALL, TURRET, SUPPORT]
        cheapest_unit = WALL
        for unit in stationary_units:
            unit_class = gamelib.GameUnit(unit, game_state.config)
            if unit_class.cost[game_state.MP] < gamelib.GameUnit(cheapest_unit, game_state.config).cost[game_state.MP]:
                cheapest_unit = unit

        # Now let's build out a line of stationary units. This will prevent our demolisher from running into the enemy base.
        # Instead they will stay at the perfect distance to attack the front two rows of the enemy base.
        for x in range(27, 5, -1):
            game_state.attempt_spawn(cheapest_unit, [x, 11])

        # Now spawn demolishers next to the line
        # By asking attempt_spawn to spawn 1000 units, it will essentially spawn as many as we have resources for
        game_state.attempt_spawn(DEMOLISHER, [24, 10], 1000)

    def least_damage_spawn_location(self, game_state, location_options):
        """
        This function will help us guess which location is the safest to spawn moving units from.
        It gets the path the unit will take then checks locations on that path to 
        estimate the path's damage risk.
        """
        damages = []
        # Get the damage estimate each path will take
        for location in location_options:
            path = game_state.find_path_to_edge(location)
            damage = 0
            for path_location in path:
                # Get number of enemy turrets that can attack each location and multiply by turret damage
                damage += len(game_state.get_attackers(path_location, 0)) * gamelib.GameUnit(TURRET, game_state.config).damage_i
            damages.append(damage)
        
        # Now just return the location that takes the least damage
        return location_options[damages.index(min(damages))]
    
    def detect_enemy_unit(self, game_state, unit_type=None, valid_x = None, valid_y = None):
        total_units = 0
        for location in game_state.game_map:
            if game_state.contains_stationary_unit(location):
                for unit in game_state.game_map[location]:
                    if unit.player_index == 1 and (unit_type is None or unit.unit_type == unit_type) and (valid_x is None or location[0] in valid_x) and (valid_y is None or location[1] in valid_y):
                        total_units += 1
        return total_units
        
    def filter_blocked_locations(self, locations, game_state):
        filtered = []
        for location in locations:
            if not game_state.contains_stationary_unit(location):
                filtered.append(location)
        return filtered

    