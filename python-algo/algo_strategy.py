import gamelib
import random
import math
import warnings
from sys import maxsize
import json
from collections import defaultdict
import numpy as np
import heapq
import random
from copy import copy
import random

"""
Most of the algo code you write will be in this file unless you create new
modules yourself. Start by modifying the 'on_turn' function.

Advanced strategy tips: 

  - You can analyze action frames by modifying on_action_frame function

  - The GameState.map object can be manually manipulated to create hypothetical 
  board states. Though, we recommended making a copy of the map to preserve 
  the actual current map state.
"""

    #* score priority:
    # respawn 1.0 
    # recycle 0.8
    # enhance 0.5
    # reinforce 0.3
    # support mp/20 
    
        

class Priority:
    def __init__(self,wall=1, turret=1,support=1):
        # priority for each unit
        self.p ={
            WALL : wall,
            TURRET: turret,
            SUPPORT: support
        }


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
            WALL: config["unitInformation"][0]["cost1"],
            SUPPORT: config["unitInformation"][1]["cost1"],
            TURRET: config["unitInformation"][2]["cost1"],
            SCOUT: config["unitInformation"][3]["cost2"],
            DEMOLISHER: config["unitInformation"][4]["cost2"],
            INTERCEPTOR: config["unitInformation"][5]["cost2"]
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
        ##key: unit's shorthand, value: [locations]
        self.owned_defenders = {
            TURRET:[[3, 12], [8, 12], [19, 12], [24, 12], [13, 11], [14, 11]],
            WALL:[[0, 13], [1,13],[2, 13], [3, 13], [8, 13], [19, 13], [24, 13], [25, 13], [26,13],[27, 13], [7, 12], [9, 12], [12, 12], [13, 12], [14, 12], [15, 12], [18, 12], [20, 12]],
            SUPPORT:[]
        }
        
        ##Defenders
        ##key: unit's shorthand, value: [locations]
        self.reinforcements = {
            WALL: [[4, 13], [5, 13], [22, 13], [23, 13], [10, 12], [17, 12],[6, 12], [11, 12], [16, 12], [21, 12],[1, 12], [2, 12], [25, 12], [26, 12], [2, 11], [25, 11]],
            TURRET: [[4, 12], [5, 12], [22, 12], [23, 12], [7, 11], [9, 11], [12, 11], [15, 11], [18, 11], [20, 11],[5, 11], [10, 11], [17, 11], [22, 11]]
        }

        ##Supports
        ##key: support area, valueL [locations]
        self.supports = {
            "init":[[8, 11], [19, 11]],
            "left_wing":[[3, 11], [4, 11]],
            "right_wing":[[23, 11], [24, 11]],
            "left_mid":[[7, 10], [8, 10], [9, 10], [10, 10]],
            "mid":[[12, 10], [13, 10], [14, 10], [15, 10]],
            "right_mid":[[17, 10], [18, 10], [19, 10], [20, 10]]
        } 
        self.support_areas = ["init", "left_wing", "right_wing","left_mid","mid", "right_mid"]
        self.curr_support_area = self.support_areas[0]

        # self.advance_reinforce = {
        #     WALL:[[1, 12], [2, 12], [25, 12], [26, 12], [2, 11], [25, 11]]
        #     TURRET:[[5, 11], [10, 11], [17, 11], [22, 11]]
        #     SUPPORT: [[3, 11], [4, 11], [23, 11], [24, 11], [7, 10], [8, 10], [9, 10], [10, 10], [12, 10], [13, 10], [14, 10], [15, 10], [17, 10], [18, 10], [19, 10], [20, 10]]
        # }
        
        global SPAWN, REMOVE, UPGRADE, NO_ACTION, NO_UNIT
        SPAWN = "spawn"
        REMOVE = "remove"
        UPGRADE = "upgrade"
        NO_ACTION = "no_action"
        NO_UNIT = "no_unit"



        # priority map and action map
        self.priority_map = np.zeros((WIDTH,WIDTH))
        self.action_map = [[(NO_ACTION, NO_UNIT)] * WIDTH for _ in range(WIDTH)]

        self.recycled_units = []
    
        #* Action priorities
        # priority for spawning hard-coded position
        self.init_priority = Priority()
        # priority for locking reserved opening position
        self.lock_priority = Priority(-100)
        # priority for re-pairing 
        self.recycle_priority = Priority()
        # priority for spawning
        self.spawn_priority = Priority()
        # priority for upgrade
        self.enhance_priority = Priority()
        # priority for reinfrocement
        self.reinforce_priority = Priority()
        #hold number of Mp to spawn 1 support
        self.support_priority = 20

        # Vulnerable range
        global VULNERABLE_RANGE
        VULNERABLE_RANGE = 2

        # attacking strategy
        self.ready_to_attack = False
        self.attack_openings = [
            [1, 13],
            [6, 12],
            [11, 12],
            [16, 12],
            [21, 12],
            [25, 13]
        ]
        self.reserved_opening = []


        
    def on_turn(self, turn_state):
        """
        This function is called every turn with the game state wrapper as
        an argument. The wrapper stores the state of the arena and has methods
        for querying its state, allocating your current resources as planned
        unit deployments, and transmitting your intended deployments to the
        game engine.
        """
        game_state = gamelib.GameState(self.config, turn_state)
        gamelib.debug_write('Performing turn {} of our strategy'.format(game_state.turn_number))
        game_state.suppress_warnings(True)  #Comment or remove this line to enable warnings.
        
        ## spawn hard coded defense only on 1st turn
        if game_state.turn_number < 1:
            self.init_defense(game_state)
        self.defense_strategy(game_state)
        ## self.starter_strategy(game_state)
        
        # deploy mobile units
        self.deploy_strategy(game_state)
        
        # submit strategy
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
        predicted_game_state = copy(game_state)
        # Returns predicted_enemy_map
        enemy_sp = game_state.get_resource(0, 1)
        norm_enemy_structure_heatmap = []
        # normalize and convert heatmap to np array
        for unit_type in [WALL, SUPPORT, TURRET]:
            norm_enemy_structure_heatmap.append(np.array(self.enemy_structure_heatmap[unit_type]) / game_state.turn_number) # list(map(lambda x: list(map(lambda: y: y / game_state.turn_number, x)), self.enemy_structure_heatmap[unit_type]))            # NOTE: very likely to have bugs
        norm_enemy_structure_heatmap = np.array(norm_enemy_structure_heatmap)    # (3x28x28)
        # filter out likely locations and sort by likelihood
        norm_enemy_structure_predict = np.argwhere(norm_enemy_structure_heatmap > possibility_bar)      # TODO: TUNABLE
        order = np.argsort(-1 * norm_enemy_structure_heatmap[norm_enemy_structure_predict])                  # need to check if this map access is correct
        norm_enemy_structure_predict = norm_enemy_structure_predict[order]
        gamelib.debug_write("Prediction on enemy structures: {}".format(norm_enemy_structure_predict))  # NOTE: for debugging
        # add likely positions to the map
        for index, x_coord, y_coord in norm_enemy_structure_predict: 
            if not predicted_game_state.can_spawn([WALL, SUPPORT, TURRET][index], [x_coord, y_coord], 1):
                continue
            enemy_sp -= self.units_cost[unit_type]
            if enemy_sp < 0:
                break
            predicted_game_state.game_map.add_unit([WALL, SUPPORT, TURRET][index], [x_coord, y_coord], 1)
        return predicted_game_state

    def predict_enemy_attack(self, game_state):
        # Returns a list of possible breach locations with corresponding probabilities
        # create normalized enemy deployment heatmap
        enemy_deploy_tensor = []
        enemy_deploy_num = []
        for unit_type in [SCOUT, DEMOLISHER, INTERCEPTOR]:
            enemy_deploy_tensor.append(self.enemy_deploy_heatmap[unit_type])
            enemy_deploy_num.append(self.enemy_deploy_num[unit_type])
        enemy_deploy_tensor = np.linalg.norm(np.array(enemy_deploy_tensor))     # (3x28)
        # filter out hot deploy locations and order by p
        enemy_deploy_location = np.argwhere(enemy_deploy_tensor > 0)                                    # TODO: TUNABLE
        #order = np.argsort(self.enemy_deploy_heatmap[enemy_deploy_location])
        #enemy_deploy_location = enemy_deploy_location[order]
        result = []
        for index, x_coord in enemy_deploy_location:
            coords = self.get_edge_location(x_coord)
            coords[1] = 27 - coords[1]  # flip Y-coordinate as it's on the enemy side
            unit_type = [SCOUT, DEMOLISHER, INTERCEPTOR][index]
            if not game_state.can_spawn(unit_type, coords, 1): continue
            num_deployed = enemy_deploy_num[index][x_coord]
            num_mode = max(set(num_deployed), key = num_deployed.count)
            tmp_state = copy(game_state)
            for i in range(num_mode):
                tmp_state.game_map.add_unit(unit_type, coords, 1)
            simulation = self.multi_attack_path_analysis(tmp_state, [coords])
            crossing_pos = simulation[7][0]
            breach_pos = simulation[6][0]
            hp_damage = simulation[3][0]
            if simulation[0] == 0:
                continue
            result.append([crossing_pos, breach_pos, enemy_deploy_tensor[index, x_coord] * hp_damage])
        result = list(sorted(result, key = lambda x: -1 * x[2]))
        gamelib.debug_write("Prediction on enemy attacks: {}".format(result))                           # NOTE: for debugging
        return result
    

    def lock_reserved_opening(self, game_state, low_priority):
        """
            set reserved opening's priority to lowest as possible 
        """
        for x,y in self.reserved_opening:
            self.priority_map[x][y] = low_priority
            self.action_map[x][y]= (REMOVE,WALL)
        self.reserved_opening = []
        return 
            
    def get_destroyed_defenders(self, game_state):
        """
            respawn destroyed defenders
            add spawn priority to priority map at x,y
            updates self.owned_defender
        """

        for unit_type in [WALL, TURRET]:
            for location in self.owned_defenders[unit_type]:
                x,y = location
                defender = game_state.game_map[location]
                #check whether defender is out of map or not
                if not defender:
                    #update priority map
                    self.priority_map[x][y] += self.spawn_priority.p[unit_type]
                    self.action_map[x][y] = (SPAWN, unit_type)
                    self.owned_defenders[unit_type].remove(location)
        return
    
    def get_damaged_defenders(self, game_state):
        """
            recycle damaged defenders
            add (recycle prioirty * health_ratio)  to priority map at x,y
            add damaged defenders to recycled list to spawn in future turn
        """
        
        defenders = [TURRET, WALL]
        for unit_type in defenders:
            for location in self.owned_defenders[unit_type]:
                x,y = location
                defender = game_state.game_map[location]
                if defender:
                    health_ratio = defender[0].health / defender[0].max_health
                    # check whether unit is damaged
                    if health_ratio != 1:
                        self.priority_map[x][y] += health_ratio * self.recycle_priority.p[unit_type]
                        self.action_map[x][y] = (REMOVE,unit_type)
                        self.recycled_units.append((location,unit_type))

        return

    def add_recycled_units(self, game_state):
        """
            add spawn priority to priority map, where x,y is the location recycled unit from previous turn
        """

        for location, unit_type in self.recycled_units:
            #check whether location in game map is occupied or not
            x,y = location
            defender = game_state.game_map[location]
            # if the current location is not occupied
            if not defender:
                self.priority_map[x][y] += self.spawn_priority.p[unit_type]
                self.action_map[x][y] = (SPAWN,unit_type)
            else:
                gamelib.debug_write("Something went wrong, recycled unit's location shouldn't be occupied") 
        return

    def get_unit_within_area(self, area, defenders,unit_type,):
        """
            filter out units, whose location is within the area
        """
        return filter(lambda x: x in area, defenders[unit_type])
        
    def enhance_vulnerable_area(self, game_state,vulnerable_locations, threateness):
        """
            enhance either vulnerable corners or vulnerable centre
        """
        defenders = [TURRET,WALL]
        vulnerable_area = game_state.game_map.get_locations_in_range(vulnerable_locations, VULNERABLE_RANGE)
        for unit_type in defenders:
            # extra all owned defenders within the area
            locations = self.get_unit_within_area(vulnerable_area, self.owned_defenders,unit_type)
            for location in locations:
                x,y = location
                defender = game_state.game_map[location]
                #make sure defender is not null
                if defender:
                    health_ratio = defender[0].health / defender[0].max_health
                    #only enhance undamaged defender
                    if health_ratio == 1:
                        self.priority_map[x][y] += threateness
        return 
        
    def enhance_defenders(self, game_state,vulnerables):
        """
            add enhance priority to priority map based on vulnerability
        """
        defenders = [WALL,TURRET]

        for unit_type in defenders:
            for location in self.owned_defenders[unit_type]:
                x,y = location
                defender = game_state.game_map[location]
                # check existence of defender and is in full health
                if defender:
                    health_ratio = defender[0].health / defender[0].max_health
                    if health_ratio == 1:
                        self.priority_map[x][y] += self.enhance_priority.p[unit_type]
                        self.action_map[x][y] = (UPGRADE,unit_type)

        # add vulnerability/threatheness to priority
        # assume location is where enemy first crosses our board        
        for crossing,breach_locations,threateness in vulnerables:
            #find vulnerable area for breaching corners
            self.enhance_vulnerable_area(game_state, breach_locations, threateness)
            # find vulnerable area for crossing
            self.enhance_vulnerable_area(game_state,crossing, threateness)     
        return

    def reinforce_vulnerable_area(self, game_state, vulnerable_locations,threateness):
        """
            reinforce either vulnerable corners or vulnerable centre
        """
        defenders = [TURRET,WALL]
        vulnerable_area = game_state.game_map.get_locations_in_range(vulnerable_locations, VULNERABLE_RANGE)
        for unit_type in defenders:
            # extra all owned defenders within the area
            locations = self.get_unit_within_area(vulnerable_area, self.reinforcements,unit_type)
            for location in locations:
                x,y = location
                unit = game_state.game_map[location]
                #if there's no unit, place reinforcement
                if not unit:
                    self.priority_map[x][y] += threateness
        return


    def reinforce_defenders(self, game_state, vulnerables):
        """
            add reinforce priority to defenders based on vulnerability
        """
        defenders = [WALL,TURRET]
        for unit_type in defenders:
            for location in self.reinforcements[unit_type]:
                defender = game_state.game_map[location]
                #make sure location is not occupied
                if not defender:
                    x,y = location
                    self.priority_map[x][y] += self.reinforce_priority.p[unit_type]
                    self.action_map[x][y] = (SPAWN,unit_type)

        # assume location is where enemy first crosses our board        
        for crossing,breach_locations, threateness in vulnerables:
            #find vulnerable area for breaching corners
            self.reinforce_vulnerable_area(game_state, breach_locations,threateness)
            # find vulnerable area for crossing
            self.reinforce_vulnerable_area(game_state, crossing,threateness)
        return

    def spawn_support_area(self,game_state, mp, area):
        """
            spawn support based on area and mp
            return True if support added to priority map successfully
        """

        remaining_supports = len(self.supports[area])
        if remaining_supports == 0:
            gamelib.debug_write("Support area is full, cannot spawn no more supports.") 
            self.support_areas.remove(area)
            return

        if self.curr_support_area == area:
            random_support = random.randint(0, remaining_supports-1)
            location = self.supports[area][random_support]
            self.supports[area].remove(location)
            x,y = location
            unit = game_state.game_map[location]
            #check whether location is occupied
            if not unit:
                self.priority_map[x][y] += mp/self.support_priority
                self.action_map[x][y] = (SPAWN, SUPPORT)
            else:
                gamelib.debug_write("Spawing supports in occupied position.") 
        else:
            gamelib.debug_write("Spawing in wrong support area.") 

        return 

    def spawn_support(self,game_state):
        """
            spawn support based on current MP
        """
        mp = game_state.MP
        # check whether the current support area is full or not
        remaining_supports = len(self.supports[self.curr_support_area])
        if remaining_supports == 0:
            self.support_areas.remove(self.curr_support_area)

        #spawn init support first
        # if current support area is fully spawned, select a different area to spawn
        # select random support area, left_wing, right_wing, left_mid, right_mid, mid
        self.curr_support_area = self.support_areas[random.randint(0, len(self.support_areas)-1)]
        self.spawn_support_area(game_state,mp,self.curr_support_area)
        return
    
    def perform_actions(self,game_state, ordered_p_map):
        """
            Assume ordered_p_map is sorted in highest priority order
            perform top priority actions
            cases to handle when spawning
                1. remove recycled location in recycled list if spawning location in recycled list
                2. updates self.owned_defenders

            cases to handle when removing:
                1. updates self.owned_defender
        """

        current_SP = game_state.SP
        i = 0 # ith action
        while current_SP > 0:
            location = ordered_p_map[i]
            x,y = location
            #extract action corresponds to x,y
            action, unit_type= self.action_map[x][y]
            if action == SPAWN:
                success = game_state.attempt_spawn(unit_type, location)
                if success > 0:
                    current_SP -= game_state.type_cost(unit_type)
                    #update self.owned_defenders
                    self.owned_defenders[unit_type].append(location)
                    # update recycled units
                    if location in self.recycled_units:
                        self.recycled_units.remove((location, unit_type))
                else:
                    gamelib.debug_write("Spawning object:{} failed".format(unit_type))  
            elif action == UPGRADE:
                    success = game_state.attempt_upgrade(location)
                    if success > 0:
                        current_SP -= game_state.type_cost(unit_type, True)
                    else:
                        gamelib.debug_write("Upgrading non existence object:{}".format(unit_type))  
            elif action == REMOVE:
                success = game_state.attempt_remove(location)
                if success > 0:
                    #update self.owned_defenders
                    self.owned_defenders[unit_type].remove(location)
                else:
                    gamelib.debug_write("Removing non existence object:{}".format(unit_type))  
            i+=1            
        return
        
    ## TODO RT
    def init_defense(self, game_state):
        """ 
            spawn defenders only on 1st turn and initializes priority map
        """
        ## attempt to spawn turrets
        success = game_state.attempt_spawn(TURRET, self.owned_defenders[TURRET])
        if not success:
            gamelib.debug_write("Forming turrets failed")

        ## attempt to spawn walls
        success =game_state.attempt_spawn(WALL, self.owned_defenders[WALL])
        if not success:
            gamelib.debug_write("Forming walls failed")
        
        return 

    def defense_strategy(self, game_state): 
        # TODO 1. re-init priority map
        #       1.1 set reserved opening to -100 and clear self.reserved_opening
        #       1.2  get all destroyed defenders,
        #       1.3 spawn recycled defenders
        #       1.4 recycle damaged defenders  * depends on health ratio
        #       1.5 enhance defenders ,depends on health ratio and vulnerability
        #       1.6 spawn reinforcement, depends on vulnerability
        #       1.7 spawn support, depends on vulnerability
        #TODO 2. sort priority in descending order
        #      2.1 perform actions

        #init priority map
        for unit_type in [WALL,TURRET, SUPPORT]:
            for location in self.owned_defenders[unit_type]:
                x,y = location
                self.priority_map[x][y] = self.init_priority.p[unit_type]
                # already spawned
                self.action_map[x][y] = (NO_ACTION,unit_type)  #! what action 2 perform
        
        vulnerables = self.predict_enemy_attack(game_state)
        
        #*1.1 lock revered opening 
        self.lock_reserved_opening(game_state, self.lock_priority.p[WALL])
        
        #*1.2 re-spawn destroyed defenders
        self.get_destroyed_defenders(game_state)
        
        #*1.3 spawn recycled units from previous turn
        self.add_recycled_units(game_state)
    
        #*1.4 get all damaged defenders
        self.get_damaged_defenders(game_state)
        
        #*1.5 upgrade/enhance defenders
        self.enhance_defenders(game_state,vulnerables)
        
        #*1.6 reinforcement
        self.reinforce_defenders(game_state, vulnerables)
        
        #*1.7  spawn supports
        self.spawn_support(game_state)

        #*2 sort priority map
        ordered_p_map = np.argsort(-1 * self.priority_map)
        
        #*2.2 perform operations
        self.perform_actions(game_state, ordered_p_map)

    def deploy_strategy(self, game_state):
        gamelib.debug_write("Begin deployment strategy")        # NOTE: for debugging
        # attacking strategy
        min_mp = 10                                                                         # TODO: Tunable
        max_mp = 20
        cur_mp = game_state.get_resource(1, 0)
        # do not deploy if mp < min_mp
        if cur_mp < min_mp and not self.ready_to_attack:
            gamelib.debug_write("Abort deployment strategy")        # NOTE: for debugging
            return
        # may choose not to deploy based on current mp
        if cur_mp / max_mp < random.random() and not self.ready_to_attack:
            gamelib.debug_write("Abort deployment strategy")        # NOTE: for debugging
            return
        # if decide to deploy, check if we need to create an opening
        no_opening = True
        for x_coord, y_coord in self.attack_openings:
            if len(game_state.game_map[[x_coord, y_coord]]) == 0:
                no_opening = False
                break
        opening_location = [[-1, -1]]
        if no_opening:
            opening_location = self.attack_openings
        # always use all mp
        possible_deploy = []
        # gamelib.debug_write("range: {}".format(int(cur_mp // self.units_cost[DEMOLISHER])))        # NOTE: for debugging
        for demolisher_num in range(0, int(cur_mp // self.units_cost[DEMOLISHER])):
            # gamelib.debug_write("demo_num: {}".format(demolisher_num))        # NOTE: for debugging
            scout_num = int((cur_mp - demolisher_num * self.units_cost[DEMOLISHER]) // self.units_cost[SCOUT])
            possible_deploy.append([scout_num, demolisher_num])
        # gamelib.debug_write("Possible deployment combo: {}".format(possible_deploy))        # NOTE: for debugging
        # compute best opportunity
        best_opportunity_pts = (-1, [-1, -1], -1. -1. -1, -1)         # (value, opening, scout_x, demolisher_x, scout_num, demolisher_num)
        best_opportunity_dmg = (-1, [-1, -1], -1. -1. -1, -1)         # (value, opening, scout_x, demolisher_x, scout_num, demolisher_num)
        best_opportunity = (-1, [-1, -1], -1, -1. -1. -1)             # (value, opening, scout_x, demolisher_x, scout_num, demolisher_num)
        for opening in opening_location:
            sim_state = copy(game_state)
            if opening[0] != -1:
                sim_state.game_map.remove_unit(opening)
            for scout_x in range(28):
                scout_location = self.get_edge_location(scout_x)
                if not sim_state.can_spawn(SCOUT, scout_location, 1): continue
                for demolisher_x in range(28):
                    demolisher_location = self.get_edge_location(demolisher_x)
                    if not sim_state.can_spawn(DEMOLISHER, demolisher_location, 1) or demolisher_x == scout_x: continue
                    for scout_num, demolisher_num in possible_deploy:
                        tmp_state = copy(sim_state)
                        for i in range(scout_num):
                            tmp_state.game_map.add_unit(SCOUT, scout_location, 0)
                        for i in range(demolisher_num):
                            tmp_state.game_map.add_unit(DEMOLISHER, demolisher_location, 0)
                        
                        simulation = self.multi_attack_path_analysis(tmp_state, [scout_location, demolisher_location])
                        points = sum(simulation[3])
                        damage = sum(simulation[2])
                        if points > best_opportunity_pts[0]:
                            best_opportunity_pts = [points, opening, scout_x, demolisher_x, scout_num, demolisher_num]
                        if damage > best_opportunity_dmg[0]:
                            best_opportunity_dmg = [damage, opening, scout_x, demolisher_x, scout_num, demolisher_num]
        # choose points-first or damage-first
        if best_opportunity_pts[0] < 4:                                                     # TODO: Tunable
            best_opportunity = best_opportunity_dmg
        elif best_opportunity_dmg[0] < 200:
            best_opportunity = best_opportunity_pts
        elif best_opportunity_pts[0] * 50 > best_opportunity_dmg[0]:
            best_opportunity = best_opportunity_pts
        else:
            best_opportunity = best_opportunity_dmg
        # if no opening, create one and deploy next turn
        if no_opening:
            self.reserved_opening = best_opportunity[1]
            game_state.attempt_remove(self.reserved_opening)
            self.ready_to_attack = True
            return
        # if has opening, spawn units
        gamelib.debug_write("Best deployment strategy: {}".format(best_opportunity))        # NOTE: for debugging
        game_state.attempt_spawn(DEMOLISHER, self.get_edge_location(best_opportunity[2]), best_opportunity[4])
        game_state.attempt_spawn(SCOUT, self.get_edge_location(best_opportunity[1]), best_opportunity[3])
        self.ready_to_attack = False

    ## RW  REFERENCE  single path, single unit type, finished
    # def single_attack_path_analysis(self, game_state, location, num, unit):
    #     # For each deployment position, try to investigate the attack path and potential result 
    #     # based on the current friendly board and analyzed/predicted enemy board.
    #     # return (can_reach, structure_demolished, damage_dealt, points_gain, first_attack_pos, breach_pos)
    #     can_reach = 0
    #     _dem_ = 0
    #     _dmg_ = 0
    #     _pnt_ = 0
    #     _loss_ = 0
    #     # add all normal supports into a set from game_state
    #     normal_supp_set = set()
    #     upgrade_supp_set = set()

    #     for i in range(14):
    #         for j in range(13-i, 13+i+1):
    #             if game_state.map[j][i][0].unit_type == SUPPORT:
    #                 if game_state.map[j][i][0].upgraded == 0:
    #                     supp_set.add((game_state.map[j][i][0].x, game_state.map[j][i][0].y))
    #                 else:
    #                     upgrade_supp_set.add((game_state.map[j][i][0].x, game_state.map[j][i][0].y))

    #     target_edge = get_target_edge(location)
    #     path = find_path_to_edge(location, target_edge)
    #     ind = 0
    #     dmg_flag = 0
    #     first_attack_pos = [-1,-1]

    #     # initialize the total health of the deployment, and accommodate the path by the speed of the unit.unit_type (in frame count)
    #     _hp_ = 0
    #     adj_path = []
    #     if unit.unit_type == SCOUT:
    #         _hp_ = num * 15
    #     elif unit.unit_type == DEMOLISHER:
    #         _hp_ = num * 5
    #         for cord in path:
    #             for i in range(2):
    #                 adj_path.append(cord)
    #     else:
    #         _hp_ = num * 40
    #         for cord in path:
    #             for i in range(4):
    #                 adj_path.append(cord)

    #     # analyze the coordinate one by one in the adjusted path (in frame manner), and at least one mobile unit exist
    #     while (ind < len(adj_path) and num > 0):
    #         cur_pos = adj_path[ind] # current coordinate

    #         # Step 0: check if we reach the target edge
    #         edge_locations = get_edge_locations(target_edge)
    #         if cur_pos in edge_locations:
    #             can_reach = 1
    #             _pnt_ = num
    #             break

    #         # Step 1: Add shields through nearby supports
    #         # normal support
    #         for nss in normal_supp_set:
    #             if nss.shieldRange > euclid_dist(nss, cur_pos):
    #                 _hp_ += 3 * num
    #                 normal_supp_set.remove(nss)
    #         # upgraded support
    #         for uss in upgrade_supp_set:
    #             if uss.shieldRange > euclid_dist(uss, cur_pos):
    #                 _hp_ += (4 + 0.3 * uss[1]) * num
    #                 normal_supp_set.remove(nss)

    #         single_unit_health = _hp_ / num # used in following code
            
    #         # Step 2: in each frame, calculate the destruction/damage that mobile units deal, and loss/health_loss of our units
    #         # Step 2.1: 淦伤害
    #         target_unit = get_target(unit)
    #         if target_unit != None:
    #             # get first attack position
    #             if dmg_flag == 0:
    #                 first_attack_pos = adj_path[ind]
    #                 dmg_flag = 1
    #             # 塔太tm硬了，没拆掉，只造成伤害
    #             if target_unit.health > num * unit.damage_f:
    #                 target_unit.health -= num * unit.damage_f
    #                 _dmg_ += num * unit.damage_f
    #             # 塔太虚了被淦掉了，但只能淦一个目标，不再需要while loop了。期间还要重新规划路径，更新gameboard
    #             else:
    #                 _dmg_ += target_unit.health
    #                 game_state.game_map[target_unit.x][target_unit.y].pop(0)
    #                 # 规划新路径
    #                 new_location = [unit.x, unit.y]
    #                 new_path = find_path_to_edge(new_location, target_edge)
    #                 adj_path = adj_path[:ind] + new_path[1:]
            
    #         # Step 2.2: 承受伤害
    #         attacking_units = get_attackers([unit.x, unit.y], 1)
    #         if len(attacking_units) > 0:
    #             # Stack total damage
    #             dmg_taking = 0
    #             for turret in attacking_units:
    #                 dmg_taking += turret.damage_i
    #             # calculate our loss
    #             next_rest_hp = _hp_ % single_unit_health
    #             # 伤害溢出，打头的mobile unit被淦掉了
    #             if next_rest_hp <= dmg_taking:
    #                 num -= 1
    #                 _loss_ += 1
    #                 _hp_ = num * single_unit_health
    #             else:
    #                 _hp_ -= dmg_taking

    #         # Step 3: Update everything
    #         ind += 1
    #         if ind < len(adj_path):
    #             unit.x = adj_path[ind][0]
    #             unit.y = adj_path[ind][1]
        

    #     # While loop is over
    #     breach_pos = [-1, -1]
    #     if can_reach == 1:
    #         breach_pos = adj_path[-1]
        
        
    #     return can_reach, _dem_, _dmg_, _pnt_, _loss_, first_attack_pos, breach_pos


    ## RW TODO DEBUG multiple paths, multiple unit types
    def multi_attack_path_analysis(self, game_state, locations):
        # For each deployment position, try to investigate the attack path and potential result
        # The investigation is simultaneous and concurrent frame by frame
        # based on the current friendly board and analyzed/predicted enemy board.
        # locations is the list of deploy locations, nums is the list of deployment numbers, units is the list of units
        # return (can_reach, structure_demolished, damage_dealt, points_gain, damage_taken, first_attack_pos, breach_pos, cross_pos)
        nums = []
        units = []
        for coord in locations:
            scout_acm_num = 0
            dem_acm_num = 0
            inter_acm_num = 0
            scout_pos = -1
            dem_pos = -1
            inter_pos = -1
            for i, un in enumerate(game_state.game_map[[coord[0], coord[1]]]):
                if un.unit_type == SCOUT:
                    scout_acm_num += 1
                    scout_pos = i
                elif un.unit_type == DEMOLISHER:
                    dem_acm_num += 1
                    dem_pos = i
                else:
                    inter_acm_num += 1
                    inter_pos = i
            if scout_acm_num > 0:
                nums.append(scout_acm_num)
                units.append(game_state.game_map[[coord[0],coord[1]]][scout_pos])
            if dem_acm_num > 0:
                nums.append(dem_acm_num)
                units.append(game_state.game_map[[coord[0], coord[1]]][dem_pos])
            if inter_acm_num > 0:
                nums.append(inter_acm_num)
                units.append(game_state.game_map[[coord[0],coord[1]]][inter_pos])
        
        #gamelib.debug_write("RW: number of different units deployed: {}".format(len(units)))
        #gamelib.debug_write("RW: number of deployment units: {}".format(len(nums)))
                
        _dem_ = 0
        _dmg_ = 0
        _pnt_ = 0
        _loss_ = 0
        breach_pos = [[-1,-1]] * len(nums)
        first_attack_pos = [[-1,-1]] * len(nums)
        cross_bd_pos = [[-1,-1]] * len(nums)

        # add all normal supports into a set from game_state
        normal_supp_set = set()
        upgrade_supp_set = set()

        for i in range(14):
            for j in range(13-i, 13+i+1):
                if len(game_state.game_map[[j,i]]) == 0:
                    continue
                if game_state.game_map[[j,i]][0].unit_type == SUPPORT:
                    if game_state.game_map[[j,i]][0].upgraded == 0:
                        normal_supp_set.add((game_state.game_map[[j,i]][0].x, game_state.game_map[[j,i]][0].y))
                    else:
                        upgrade_supp_set.add((game_state.game_map[[j,i]][0].x, game_state.game_map[[j,i]][0].y))
        
        #gamelib.debug_write("RW: support set created")

        target_edges = []
        for loc in locations:
            target_edges.append(game_state.get_target_edge(loc))
        paths = []
        for i in range(len(units)):
            paths.append(game_state.find_path_to_edge(locations[i], target_edges[i]))
        
        adj_paths = []
        for i in range(len(units)):
            adj_path = []
            if units[i].unit_type == SCOUT:
                adj_paths.append(paths[i])
            elif units[i].unit_type == DEMOLISHER:
                for cord in paths[i]:
                    for j in range(2):
                        adj_path.append(cord)
                adj_paths.append(adj_path)
            else:
                for cord in paths[i]:
                    for i in range(4):
                        adj_path.append(cord)
                adj_paths.append(adj_path)

        #gamelib.debug_write("RW: adjusted paths set up")

        list_normal_supp_set = []
        list_upgrade_supp_set = []
        for i in range(len(adj_paths)):
            list_normal_supp_set.append(normal_supp_set)
            list_upgrade_supp_set.append(upgrade_supp_set)
        
        total_num = sum(nums)
        ind = 0
        dmg_flag = [0] * len(units)
        full_health = [0] * len(units)
        #gamelib.debug_write("RW: all set up, simulation starts here")
        # analyze the coordinate one by one in the adjusted path (in frame manner), and at least one mobile unit exist
        while (total_num > 0):
            update_SP = {}
            
            for i, adj_path in enumerate(adj_paths):
                # This path has been exhausted
                if ind >= len(adj_paths):
                    continue
                
                cur_pos = adj_path[ind] # current coordinate
                if cur_pos[1] == 14:
                    cross_bd_pos[i] = cur_pos

                # Step 0: check if we reach the target edge
                edge_locations = game_state.game_map.get_edge_locations(target_edges[i])
                if cur_pos in edge_locations:
                    can_reach = 1
                    breach_pos[i] = cur_pos
                    _pnt_ += nums[i]
                    continue

                #gamelib.debug_write("RW: single path loop step 0 finished, seems no points scored, all good")

                # Step 1: Add shields through nearby supports
                # normal support
                for nss in normal_supp_set:
                    if nss.shieldRange > self.euclid_dist(nss, cur_pos):
                        units[i].health += 3
                        list_normal_supp_set[i].remove(nss)
                # upgraded support
                for uss in upgrade_supp_set:
                    if uss.shieldRange > self.euclid_dist(uss, cur_pos):
                        units[i].health += 4 + 0.3 * uss[1]
                        list_normal_supp_set[i].remove(nss)
                        
                if full_health[i] < units[i].health:
                    full_health[i] = units[i].health
                
                #gamelib.debug_write("RW: single path loop step 1 finished, shields have been added, all good")

                # Step 2: in each frame, calculate the destruction/damage that mobile units deal, and loss/health_loss of our units
                # Step 2: 淦伤害
                target_unit = game_state.get_target(units[i])
                if target_unit != None:
                    # get first attack position
                    if dmg_flag[i] == 0:
                        first_attack_pos[i] = adj_path[ind]
                        dmg_flag[i] = 1
                    # 计算该单位对 target 的伤害, 记录伤害，之后update game_state，update path
                    if (target_unit.x, target_unit.y) in update_SP:
                        update_SP[(target_unit.x, target_unit.y)] += units[i].damage_f * nums[i]
                    else:
                        update_SP[(target_unit.x, target_unit.y)] = units[i].damage_f * nums[i]
                
                #gamelib.debug_write("RW: single path loop step 2 finished, all damage dealt has been calculated, all good")
            
                # Step 3: 承受伤害
                attacking_units = game_state.get_attackers([units[i].x, units[i].y], 1)
                if len(attacking_units) > 0:
                    # Stack total damage
                    dmg_taking = 0
                    for turret in attacking_units:
                        dmg_taking += turret.damage_i
                    # 伤害溢出，打头的mobile unit被淦掉了
                    if units[i].health <= dmg_taking:
                        nums[i] -= 1
                        total_num -= 1
                        _loss_ += 1
                        # 刷新，下一个 mobile unit 出现
                        units[i].health = full_health[i]
                    # 伤害没益处，扣掉现在打头mobile unit的血量
                    else:
                        units[i].health -= dmg_taking
                
                #gamelib.debug_write("RW: single path loop step 3 finished, all damage taken has been processed, all good")
        
            # For loop is over
            # First update the game_state
            for loc in update_SP:
                if game_state.game_map[[loc[0],loc[1]]][0].health > update_SP[loc]:
                    game_state.game_map[[loc[0],loc[1]]][0].health -= update_SP[loc]
                    _dmg_ += update_SP[loc]
                else:
                    _dmg_ += game_state.game_map[[loc[0],loc[1]]][0].health
                    game_state.game_map[[loc[0],loc[1]]][0] = []
                    _dem_ += 1
            # Then update the adj_paths
            for i, path in enumerate(adj_paths):
                if ind >= len(path):
                    continue
                temp_path = game_state.find_path_to_edge([units[i].x, units[i].y], target_edges[i])
                adj_paths[i] = adj_paths[i][:ind] + temp_path[ind:]
            # Then update the unit location
            for i, unit in enumerate(units):
                if ind+1 >= len(adj_paths[i]):
                    continue
                units[i].x = adj_paths[i][ind+1][0]
                units[i].y = adj_paths[i][ind+1][1]
            # Finally, update the ind to ind+1
            ind += 1

            #gamelib.debug_write("RW: single frame for loop step finished, all perfect")
            
        # While loop is over
        #gamelib.debug_write("RW: main while loop finished, all done")
        
        
        return can_reach, _dem_, _dmg_, _pnt_, _loss_, first_attack_pos, breach_pos, cross_bd_pos
        

    """
    ----------------------------helper functions----------------------------
    """
    
    def get_edge_location(self, x):
        # return edge coordinates given x
        return [x, abs(math.floor(13.5-x))]
    
    def euclid_dist(self, loc1, loc2):
        dist = math.sqrt(pow(loc1[0] - loc2[0], 2) + pow(loc1[1] - loc2[1], 2))
        return dist

if __name__ == "__main__":
    algo = AlgoStrategy()
    algo.start()
