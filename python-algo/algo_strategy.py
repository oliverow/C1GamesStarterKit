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
from copy import deepcopy
import random

# //                              _ooOoo_
# //                             o8888888o
# //                             88" . "88
# //                             (| -_- |)
# //                              O\ = /O
# //                           ____/`---'\____
# //                        .   ' \\| |// `.
# //                         / \\||| : |||// \
# //                        / _||||| -:- |||||- \
# //                         | | \\\ - /// | |
# //                       | \_| ''\---/'' | |
# //                        \ .-\__ `-` ___/-. /
# //                    ___`. .' /--.--\ `. . __
# //                  ."" '< `.___\_<|>_/___.' >'"".
# //                 | | : `- \`.;`\ _ /`;.`/ - ` : | |
# //                    \ \ `-. \_ __\ /__ _/ .-` / /
# //           ======`-.____`-.___\_____/___.-`____.-'======
# //                              `=---='
# //
# //           .............................................
# //                     佛祖保佑             永无BUG
# //            佛曰:
# //                     写字楼里写字间，写字间里程序员；
# //                     程序人员写程序，又拿程序换酒钱。
# //                     酒醒只在网上坐，酒醉还来网下眠；
# //                     酒醉酒醒日复日，网上网下年复年。
# //                     但愿老死电脑间，不愿鞠躬老板前；
# //                     奔驰宝马贵者趣，公交自行程序员。
# //                     别人笑我忒疯癫，我笑自己命太贱；
# //                     不见满街漂亮妹，哪个归得程序员？

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
        self.enemy_structure_prev = np.zeros((WIDTH, WIDTH))
        self.enemy_structure_heatmap = {
            WALL: np.zeros((WIDTH, WIDTH)),
            SUPPORT: np.zeros((WIDTH, WIDTH)),
            TURRET: np.zeros((WIDTH, WIDTH))
        }
        self.enemy_deploy_heatmap = {
            SCOUT: [0] * WIDTH,
            DEMOLISHER: [0] * WIDTH,
            INTERCEPTOR: [0] * WIDTH
        }
        self.enemy_deploy_num = {
            SCOUT: [[] for _ in range(WIDTH)],
            DEMOLISHER: [[] for _ in range(WIDTH)],
            INTERCEPTOR: [[] for _ in range(WIDTH)]
        }

        ##Defenders
        ##key: unit's shorthand, value: [locations]

        global PINK, YELLOW, BLUE
        PINK = "pink"
        BLUE = "blue"
        YELLOW="yellow"

        
        self.all_structures = {
            PINK:set([(0, 13), (2, 13), (3, 13), (8, 13), (19, 13), (24, 13), (25, 13), (27, 13), (7, 12), (9, 12), (12, 12), (13, 12), (14, 12), (15, 12), (18, 12), (20, 12),(1, 13), (26, 13),(3, 12), (8, 12), (19, 12), (24, 12), (13, 11), (14, 11)]),
            BLUE:set([(4, 13), (5, 13), (22, 13), (23, 13), (10, 12), (17, 12),(6, 12), (11, 12), (16, 12), (21, 12),(4, 12), (5, 12), (22, 12), (23, 12), (7, 11), (9, 11), (12, 11), (15, 11), (18, 11), (20, 11)]),
            YELLOW:set([(1, 12), (2, 12), (25, 12), (26, 12), (2, 11), (25, 11),(3, 11), (4, 11), (23, 11), (24, 11), (7, 10), (8, 10), (9, 10), (10, 10), (12, 10), (13, 10), (14, 10), (15, 10), (17, 10), (18, 10), (19, 10), (20, 10),(5, 11), (10, 11), (17, 11), (22, 11)])
        }

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

        # self.support_areas = SupportAreas(supports)
        self.support_areas = ["init", "left_wing", "right_wing","left_mid","mid", "right_mid"]
        self.curr_support_area = self.support_areas[0]
        self.completed_area = 0



        global SPAWN, REMOVE, UPGRADE, NO_ACTION, NO_UNIT
        SPAWN = "spawn"
        REMOVE = "remove"
        UPGRADE = "upgrade"
        NO_ACTION = "no_action"
        NO_UNIT = "no_unit"



        # priority map and action map
        self.priority_map = np.zeros((WIDTH,WIDTH))
        self.action_map = [[(NO_ACTION, NO_UNIT)] * WIDTH for _ in range(WIDTH)]

        self.recycled_units = set()
    
        #* Action priorities
        # priority for spawning hard-coded position
        # self.init_priority = Priority(1.3,1.3)

        self.based_priorities = {
            PINK: Priority(12,12),
            BLUE: Priority(0.8,0.8),
            YELLOW:Priority(0.4,0.4)
        }

        # priority for locking reserved opening position
        self.lock_priority = Priority(-100)
        # priority for re-pairing 
        self.recycle_priority = Priority(1, 0.8, 0)
        # priority for re-spawning recycled or destroyed
        # self.spawn_priority = Priority(1,1,1)

        # priority for upgrade
        self.enhance_priority = Priority(0.7,1.6,0.9)
        # priority for reinfrocement
        self.reinforce_priority = Priority(0.3,0.5,0.5)
        #hold number of Mp to spawn 1 support
        self.support_priority = 8

        # Vulnerable range
        global VULNERABLE_RANGE
        VULNERABLE_RANGE = 2
        self.crossing_locations = []

        # attacking strategy
        self.ready_to_attack = False
        self.attack_openings = [
            [6, 12],
            [11, 12],
            [16, 12],
            [21, 12]
        ]
        self.attack_deploy_xpos = [1, 3, 5, 7, 20, 22, 24, 26]
        self.reserved_opening = []
        self.reserved_opportunity = [-1, [-1, -1], -1, -1, -1, -1]

        
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
        
        # deploy mobile units
        self.deploy_strategy(game_state)

        # reset enemy analysis flag
        self.first_frame_captured = False
        # reset crossing locations
        self.crossing_locations = []
        # reset revered_opening
        # self.reserved_opening = []


        gamelib.debug_write('Submitting turn {} of our strategy'.format(game_state.turn_number))
        # import time; time.sleep(10)


        # submit strategy
        game_state.submit_turn()

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
        # check when enemy crossed border
        for move in state['events']['move']:
            # gamelib.debug_write('move event {}'.format(move))
            if move[5] == 1: continue
            if move[0][1] == 14 and move[1][1] == 13:
                self.crossing_locations.append(move[1])

        # for breach in state['events']["breach"]:
        #     location = breach[0]
        #     unit_owner_self = True if breach[4] == 1 else False
        #     # When parsing the frame data directly, 
        #     # 1 is integer for yourself, 2 is opponent (StarterKit code uses 0, 1 as player_index instead)
        #     if not unit_owner_self:
        #         gamelib.debug_write("Got scored on at: {}".format(location))
        #         self.scored_on_locations.append(location)
        #         gamelib.debug_write("All locations: {}".format(self.scored_on_locations))

    """
    ------------------------------------------Our code below:
    """
    def analyze_enemy(self, frame_state):
        # A helper to track enemy moves for enemy analysis
        enemy_structure_current = np.zeros((WIDTH, WIDTH))
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
                for x_coord, num in unit_num.items():
                    self.enemy_deploy_heatmap[unit_type][x_coord] += num
                    self.enemy_deploy_num[unit_type][x_coord].append(num)
        self.enemy_structure_prev = enemy_structure_current

    def predict_enemy_structure(self, game_state, possibility_bar = 0.75):
        predicted_game_state = deepcopy(game_state)
        # Returns predicted_enemy_map
        enemy_sp = game_state.get_resource(0, 1)
        norm_enemy_structure_heatmap = []
        # normalize and convert heatmap to np array
        for unit_type in [WALL, SUPPORT, TURRET]:
            norm_enemy_structure_heatmap.append(np.array(self.enemy_structure_heatmap[unit_type]) / (game_state.turn_number+1)) # list(map(lambda x: list(map(lambda: y: y / game_state.turn_number, x)), self.enemy_structure_heatmap[unit_type]))            # NOTE: very likely to have bugs
        norm_enemy_structure_heatmap = np.array(norm_enemy_structure_heatmap)    # (3x28x28)
        # gamelib.debug_write("heatmap: {}".format(norm_enemy_structure_heatmap[2].max()))  # NOTE: for debugging
        # filter out likely locations and sort by likelihood
        norm_enemy_structure_predict = np.argwhere(norm_enemy_structure_heatmap > possibility_bar)      # TODO: TUNABLE
        order = np.argsort(-1 * norm_enemy_structure_heatmap[norm_enemy_structure_predict.T[0], norm_enemy_structure_predict.T[1], norm_enemy_structure_predict.T[2]])                  # need to check if this map access is correct
        norm_enemy_structure_predict = norm_enemy_structure_predict[order]
        # gamelib.debug_write("Prediction on enemy structures: {}".format(norm_enemy_structure_predict))  # NOTE: for debugging
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
        game_state = self.predict_enemy_structure(game_state)
        enemy_deploy_tensor = []
        enemy_deploy_num = []
        for unit_type in [SCOUT, DEMOLISHER, INTERCEPTOR]:
            enemy_deploy_tensor.append(self.enemy_deploy_heatmap[unit_type])
            enemy_deploy_num.append(self.enemy_deploy_num[unit_type])
        enemy_deploy_tensor = np.array(enemy_deploy_tensor)         # (3x28)
        norm = np.linalg.norm(enemy_deploy_tensor)
        if norm: enemy_deploy_tensor = enemy_deploy_tensor / norm
        # filter out hot deploy locations and order by p
        enemy_deploy_location = np.argwhere(enemy_deploy_tensor > 0.2)                                    # TODO: TUNABLE
        #order = np.argsort(self.enemy_deploy_heatmap[enemy_deploy_location])
        #enemy_deploy_location = enemy_deploy_location[order]
        result = []
        for index, x_coord in enemy_deploy_location:
            coords = self.get_edge_location(x_coord)
            coords[1] = 27 - coords[1]
            unit_type = [SCOUT, DEMOLISHER, INTERCEPTOR][index]
            if len(game_state.game_map[coords]): continue
            num_deployed = enemy_deploy_num[index][x_coord]
            num_mode = max(set(num_deployed), key = num_deployed.count)
            tmp_state = deepcopy(game_state)
            for i in range(num_mode):
                tmp_state.game_map.add_unit(unit_type, coords, 1)
            # gamelib.debug_write("Prediction on enemy attacks: {} {}".format(unit_type, coords))                           # NOTE: for debugging
            simulation = self.multi_attack_path_analysis(tmp_state, [coords])
            crossing_pos = simulation[7][0]
            breach_pos = simulation[6][0]
            sp_damage = simulation[2]
            hp_damage = simulation[3]
            if simulation[0] == 0:
                continue
            result.append([crossing_pos, breach_pos, enemy_deploy_tensor[index, x_coord] * (hp_damage + sp_damage / 2)])
        result = list(sorted(result, key = lambda x: -1 * x[2]))
        # gamelib.debug_write("Prediction on enemy attacks: {}".format(result))                           # NOTE: for debugging
        return result
    

    def lock_reserved_opening(self, game_state, low_priority):
        """
            set reserved opening's priority to lowest as possible
        """
        for x, y in self.reserved_opening:
            self.priority_map[x][y] = low_priority
            self.action_map[x][y] = (REMOVE, WALL)
        self.reserved_opening = []
        return
    
    def is_corner(self, x,y):
        """
            determine whether the given location is within the 2 corners
        """
        return 10 < y <= 13 and (0 <= x <3 or 24<= x <=27)

    def get_destroyed_defenders(self, game_state):
        """
            respawn destroyed defenders
            add spawn priority to priority map at x,y
            updates self.owned_defender
        """

        for unit_type in [WALL, TURRET]:
            for location in self.owned_defenders[unit_type]:
                x, y = location
                defender = game_state.game_map[location]
                # check whether defender is out of map or not
                if not defender:
                    # update priority map
                    # add breached priority to breached corners
                    destroyed_priority = 0
                    if self.is_corner(x,y) :
                        destroyed_priority = y * 0.5
                    for base in [PINK, BLUE, YELLOW]:
                        if (x,y) in self.all_structures[base]:
                            self.priority_map[x][y] += self.based_priorities[base].p[unit_type] + destroyed_priority
                            # gamelib.debug_write("Destroyed P Map x:{}, y:{}, base:{}, priority:{}".format(x,y,base,self.priority_map[x][y]))
                    self.action_map[x][y] = (SPAWN, unit_type)
                    self.owned_defenders[unit_type].remove(location)
        return
    
    def get_damaged_defenders(self, game_state):
        """
            recycle damaged defenders
            add (recycle priority * health_ratio)  to priority map at x,y
            add damaged defenders to recycled list to spawn in future turn
        """

        defenders = [TURRET, WALL]
        for unit_type in defenders:
            for location in self.owned_defenders[unit_type]:
                x, y = location
                defender = game_state.game_map[location]
                if defender:
                    health_ratio = defender[0].health / defender[0].max_health
                    # check whether unit is damaged
                    if (health_ratio < 0.8 and not defender[0].upgraded) or (health_ratio < 0.7 and defender[0].upgraded):
                        self.priority_map[x][y] += health_ratio * self.recycle_priority.p[unit_type]
                        # gamelib.debug_write("Remove P Map x:{}, y:{}, priority:{}".format(x,y,self.priority_map[x][y]))
                        self.action_map[x][y] = (REMOVE, unit_type)
        return

    def add_recycled_units(self, game_state):
        """
            add spawn priority to priority map, where x,y is the location recycled unit from previous turn
        """

        for location, unit_type in self.recycled_units:
            # check whether location in game map is occupied or not
            x, y = location
            defender = game_state.game_map[location]
            # if the current location is not occupied
            if not defender:
                for base in [PINK, BLUE, YELLOW]:
                    if (x,y) in self.all_structures[base]:
                        self.priority_map[x][y] += self.based_priorities[base].p[unit_type]
                        gamelib.debug_write("Respawn recycled P Map x:{}, y:{}, base:{}, priority:{}".format(x,y,base,self.priority_map[x][y]))
                self.action_map[x][y] = (SPAWN, unit_type)
            else:
                gamelib.debug_write("Something went wrong, x:{}, y:{} shouldn't be occupied".format(x, y))
        return

    def get_unit_within_area(self, area, defenders, unit_type,):
        """
            filter out units, whose location is within the area
        """
        return filter(lambda x: x in area, defenders[unit_type])
        
    def enhance_vulnerable_area(self, game_state, vulnerable_locations, threateness):
        """
            enhance either vulnerable corners or vulnerable centre
        """
        defenders = [TURRET, WALL]
        vulnerable_area = game_state.game_map.get_locations_in_range(
            vulnerable_locations, VULNERABLE_RANGE)
        # gamelib.debug_write("Vulnerable breached area: {}".format(vulnerable_area))
        for unit_type in defenders:
            # extract all owned defenders within the area
            locations = self.get_unit_within_area(
                vulnerable_area, self.owned_defenders, unit_type)
            for location in locations:
                x, y = location
                defender = game_state.game_map[location]
                # make sure defender is not null
                if defender:
                    health_ratio = defender[0].health / defender[0].max_health
                    # only enhance undamaged defender
                    if health_ratio == 1:
                        self.priority_map[x][y] += threateness
        return 
        
    def enhance_defenders(self, game_state, vulnerables):
        """
            add enhance priority to priority map based on vulnerability
        """
        defenders = [WALL, TURRET]

        for unit_type in defenders:
            for location in self.owned_defenders[unit_type]:
                x, y = location
                defender = game_state.game_map[location]
                # check existence of defender and is in full health
                if defender:
                    health_ratio = defender[0].health / defender[0].max_health
                    if health_ratio == 1:
                        self.priority_map[x][y] += self.enhance_priority.p[unit_type] + y*0.01
                        # gamelib.debug_write("Enhance P Map x:{}, y:{}, priority:{}".format(x,y,self.priority_map[x][y]))
                        self.action_map[x][y] = (UPGRADE, unit_type)
                        # gamelib.debug_write(
                        #     "before enhance p map location:{}, x:{}, y:{}".format(self.priority_map[x][y], x, y))

        # add vulnerability/threatheness to priority
        # assume location is where enemy first crosses our board
        for crossing, breach_locations, threateness in vulnerables:
            # find vulnerable area for breaching corners
            self.enhance_vulnerable_area(
                game_state, breach_locations, threateness)
            # find vulnerable area for crossing
            self.enhance_vulnerable_area(game_state, crossing, threateness)
        return

    def enhance_support(self, game_state, high_priority):
        """
            Add high priority to upgrade supports 
        """
        if self.completed_area > 1:
            high_priority +=1
        elif self.completed_area > 2:
            high_priority +=1.2
        if self.curr_support_area != "init":
            for location in self.owned_defenders[SUPPORT]:
                x,y = location
                support = game_state.game_map[location]
                #check existence of support 
                if support:
                    self.priority_map[x][y] += self.enhance_priority.p[SUPPORT] + high_priority
                    self.action_map[x][y] = (UPGRADE, SUPPORT)
        return
        
    def improve_frontline_defense(self, game_state):
        """
            improve front line defense
        """
        for crossing in self.crossing_locations:
            vulnerable_area = game_state.game_map.get_locations_in_range(crossing, VULNERABLE_RANGE)
            for x,y in vulnerable_area:
                self.priority_map[x][y] += 3
        return 
        

    def reinforce_vulnerable_area(self, game_state, vulnerable_locations, threateness):
        """
            reinforce either vulnerable corners or vulnerable centre
        """
        defenders = [TURRET, WALL]
        vulnerable_area = game_state.game_map.get_locations_in_range(
            vulnerable_locations, VULNERABLE_RANGE)
        
        for unit_type in defenders:
            # extra all owned defenders within the area
            locations = self.get_unit_within_area(
                vulnerable_area, self.reinforcements, unit_type)
            for location in locations:
                x, y = location
                unit = game_state.game_map[location]
                # if there's no unit, place reinforcement
                if not unit:
                    self.priority_map[x][y] += threateness
        return

    def reinforce_defenders(self, game_state, vulnerables):
        """
            add reinforce priority to defenders based on vulnerability
        """
        defenders = [WALL, TURRET]
        for unit_type in defenders:
            for location in self.reinforcements[unit_type]:
                defender = game_state.game_map[location]
                # make sure location is not occupied
                if not defender:
                    x, y = location
                    self.priority_map[x][y] += self.reinforce_priority.p[unit_type]
                    # gamelib.debug_write("Reinforce P Map x:{}, y:{}, priority:{}".format(x,y,self.priority_map[x][y]))
                    self.action_map[x][y] = (SPAWN, unit_type)

        # assume location is where enemy first crosses our board
        for crossing, breach_locations, threateness in vulnerables:
            # find vulnerable area for breaching corners
            self.reinforce_vulnerable_area(
                game_state, breach_locations, threateness)
            # find vulnerable area for crossing
            self.reinforce_vulnerable_area(game_state, crossing, threateness)
        return

    def spawn_support_area(self, game_state, mp, area):
        """
            spawn support based on area and mp
            return True if support added to priority map successfully
        """

        remaining_supports = len(self.supports[area])
        if remaining_supports == 0:
            gamelib.debug_write(
                "Support area is full, cannot spawn no more supports.")
            self.support_areas.remove(area)
            return

        if self.curr_support_area == area:
            random_support = random.randint(0, remaining_supports-1)
            location = self.supports[area][random_support]
            # self.supports[area].remove(location)
            x, y = location
            unit = game_state.game_map[location]
            # check whether location is occupied
            if not unit:
                self.priority_map[x][y] += mp/self.support_priority
                # gamelib.debug_write("Spawn P Map x:{}, y:{}, priority:{}".format(x,y,self.priority_map[x][y]))
                self.action_map[x][y] = (SPAWN, SUPPORT)
            # else:
                # gamelib.debug_write("Spawing supports in occupied position.")
        # else:
            # gamelib.debug_write("Spawing in wrong support area.")

        return

    def spawn_support(self, game_state):
        """
            spawn support based on current MP
        """
        mp = game_state.get_resource(0, 0)

        # check whether the current support area is full or not
        remaining_supports = len(self.supports[self.curr_support_area])
        # gamelib.debug_write(
        #     "current_support:{}".format(self.curr_support_area))
        if remaining_supports == 0:
            # spawn init support first
            # if current support area is fully spawned, select a different area to spawn
            # select random support area, left_wing, right_wing, left_mid, right_mid, mid
            n = len(self.support_areas)
            if  n == 1:
                return
            _id = random.randint(0, n-1)
            self.curr_support_area = self.support_areas[_id]
            self.completed_area +=1
            return

        self.spawn_support_area(game_state, mp, self.curr_support_area)
        return
    
    def perform_actions(self, game_state, ordered_p_map):
        """
            Assume ordered_p_map is sorted in highest priority order
            perform top priority actions
            cases to handle when spawning
                1. remove recycled location in recycled list if spawning location in recycled list
                2. updates self.owned_defenders
                3. remove location in current support area, if spawning support in current support area

            cases to handle when removing:
                1. updates self.owned_defender
        """
        current_SP = game_state.get_resource(0, 0)

        i = 0  # ith action
        while current_SP > 0 and i < 784:
            location = list(ordered_p_map[i])
            x, y = location
            # extract action corresponds to x,y
            action, unit_type = self.action_map[x][y]
            if action == SPAWN:
                success = game_state.attempt_spawn(unit_type, [location])
                if success > 0:
                    current_SP -= game_state.type_cost(unit_type)[0]
                    # update self.owned_defenders
                    self.owned_defenders[unit_type].append(location)
                    # update recycled units
                    if (tuple(location), unit_type) in self.recycled_units:
                        self.recycled_units.remove(
                            (tuple(location), unit_type))
                    # check whether spawning unit is a support:
                    if unit_type == SUPPORT:
                        # gamelib.debug_write(
                        #     "current_support spawned:{}".format(self.curr_support_area))
                        self.supports[self.curr_support_area].remove(location)

                    # gamelib.debug_write(
                    #     "Spawned object: location:{}, object:{}".format(location, unit_type))

            elif action == UPGRADE:
                success = game_state.attempt_upgrade([location])
                if success > 0:
                    current_SP -= game_state.type_cost(unit_type, True)[0]
                    # gamelib.debug_write(
                    #     "Upgraded object: location:{}, object:{}".format(location, unit_type))

            elif action == REMOVE:
                success = game_state.attempt_remove([location])
                if success > 0:
                    # update self.owned_defenders
                    self.owned_defenders[unit_type].remove(location)
                    self.recycled_units.add((tuple(location), unit_type))
                    gamelib.debug_write("recycled: {}".format(self.recycled_units))

                    gamelib.debug_write(
                        "Removed object: location:{}, object:{}".format(location, unit_type))
            i += 1
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
        #       1.6 enhance supports
        #       1.7 spawn reinforcement, depends on vulnerability
        #       1.8 spawn support, depends on vulnerability
        # TODO 2. sort priority in descending order
        #      2.1 perform actions

        # NOTE: for debugging
        gamelib.debug_write("Begin defense strategy")

        # init priority map
        for unit_type in [WALL, TURRET, SUPPORT]:
            for location in self.owned_defenders[unit_type]:
                x, y = location
                for base in [PINK, BLUE, YELLOW]:
                    if (x,y) in self.all_structures[base]:
                        self.priority_map[x][y] = self.based_priorities[base].p[unit_type]
                        # gamelib.debug_write("Init P Map base:{}, priority:{}".format(base,self.priority_map[x][y]))
                        # gamelib.debug_write("Init P Map x:{}, y:{}, priority:{}".format(x,y,self.priority_map[x][y]))
                self.action_map[x][y] = (NO_ACTION, unit_type)

        vulnerables = self.predict_enemy_attack(game_state)
        # gamelib.debug_write("Vulnerables: {}".format(vulnerables))
        # *1.1 lock revered opening
        self.lock_reserved_opening(game_state, self.lock_priority.p[WALL])

        # *1.2 re-spawn destroyed defenders
        self.get_destroyed_defenders(game_state)

        # *1.3 get all damaged defenders
        self.get_damaged_defenders(game_state)
        # gamelib.debug_write("recycled: {}".format(self.recycled_units))
        
        # *1.4 spawn recycled units from previous turn
        self.add_recycled_units(game_state)

        # *1.5 upgrade/enhance defenders
        self.enhance_defenders(game_state, vulnerables)
        
        # *1.6 enhace supports
        self.enhance_support(game_state,0.1)

        # *1.7 reinforcement
        self.reinforce_defenders(game_state, vulnerables)

        ##* Improve frontline defense
        self.improve_frontline_defense(game_state)

        # *1.8  spawn supports
        self.spawn_support(game_state)

        # *2 sort priority map
        ordered_p_map = np.dstack(np.unravel_index(
            np.argsort(-1 * self.priority_map.ravel()), self.priority_map.shape))

        # *2.1 perform operations
        self.perform_actions(game_state, ordered_p_map[0])
        return

    def deploy_strategy(self, game_state):
        gamelib.debug_write("Begin deployment strategy")        # NOTE: for debugging
        # attacking strategy
        min_mp = 13                                                                         # TODO: Tunable
        max_mp = 21
        cur_mp = game_state.get_resource(1, 0)
        if self.ready_to_attack:
            cur_mp -= 1
        # do not deploy if mp < min_mp
        if cur_mp < min_mp and not self.ready_to_attack:
            gamelib.debug_write("Abort deployment")        # NOTE: for debugging
            return
        # may choose not to deploy based on current mp
        if cur_mp / max_mp < random.random() and not self.ready_to_attack:
            gamelib.debug_write("Abort deployment")        # NOTE: for debugging
            return
        pred_game_state = self.predict_enemy_structure(game_state)
        # if decide to deploy, check if we need to create an opening
        no_opening = True
        for x_coord, y_coord in self.attack_openings:
            if len(game_state.game_map[[x_coord, y_coord]]) == 0:
                no_opening = False
                break
        opening_locations = [[-1, -1]]
        if no_opening:
            opening_locations = self.attack_openings
        # compute best opportunity
        best_opportunity_pts = [-1, [-1, -1], -1, -1, -1, -1]         # (value, opening, scout_x, demolisher_x, scout_num, demolisher_num)
        best_opportunity_dmg = [-1, [-1, -1], -1, -1, -1, -1]         # (value, opening, scout_x, demolisher_x, scout_num, demolisher_num)
        best_opportunity = [-1, [-1, -1], -1, -1, -1, -1]             # (value, opening, scout_x, demolisher_x, scout_num, demolisher_num)
        # if there is pre-determined opportunity, skip all judgement
        if self.reserved_opportunity[0] == -1:
            # decide attack mode (duel or singular)
            deploy_both = False
            hostile_turrets = self.get_enemy_turrets_on_board(pred_game_state)
            if hostile_turrets < 5:
                deploy_both = True
            # always use all mp
            possible_deploy = []
            # gamelib.debug_write("range: {}".format(int(cur_mp // self.units_cost[DEMOLISHER])))        # NOTE: for debugging
            top = int(cur_mp // self.units_cost[DEMOLISHER])
            step = -1 * math.ceil(top / 2)
            for demolisher_num in range(top, 0, step):         # TODO: Tunable
                # gamelib.debug_write("demo_num: {}".format(demolisher_num))        # NOTE: for debugging
                if deploy_both:
                    scout_num = int((cur_mp - demolisher_num * self.units_cost[DEMOLISHER]) // self.units_cost[SCOUT])
                    possible_deploy.append([scout_num, demolisher_num])
                else:
                    scout_num = 0
                    possible_deploy.append([scout_num, demolisher_num])
                    break
            # gamelib.debug_write("Possible deployment combo: {}".format(possible_deploy))        # NOTE: for debugging
            # make configurations:
            configs = []
            for i in range(0, 6):                                                                  # TODO: Tunable
                attack_deploy_xpos = []
                for xpos in self.attack_deploy_xpos:
                    if game_state.can_spawn(SCOUT, self.get_edge_location(xpos)):
                        attack_deploy_xpos.append(xpos)
                # gamelib.debug_write("deploy strategies: {}".format(attack_deploy_xpos))        # NOTE: for debugging
                x1 = random.randint(0, len(attack_deploy_xpos)-1)
                demolisher_x = attack_deploy_xpos[x1]
                x2 = x1
                while x2 == x1 and len(attack_deploy_xpos) > 1:
                    x2 = random.randint(0, len(attack_deploy_xpos)-1)
                scout_x = self.attack_deploy_xpos[x2]
                opening = opening_locations[random.randint(0, len(opening_locations)-1)]
                configs.append([scout_x, demolisher_x, opening])
            
            for config in configs:
                gamelib.debug_write("config strategies: {}".format(config))        # NOTE: for debugging
                scout_x, demolisher_x, opening = config
                # process opening
                # gamelib.debug_write("opening: {}".format(opening))        # NOTE: for debugging
                sim_state = deepcopy(pred_game_state)
                if opening[0] != -1:                                                                # remove opening if necessary
                    sim_state.game_map.remove_unit(opening)
                # gamelib.debug_write("scout x: {}".format(scout_x))        # NOTE: for debugging
                scout_location = self.get_edge_location(scout_x)
                # gamelib.debug_write("demo x: {}".format(demolisher_x))        # NOTE: for debugging
                demolisher_location = self.get_edge_location(demolisher_x)
                
                for scout_num, demolisher_num in possible_deploy:                           # for each possible deploy combo
                    demolisher_location = self.get_edge_location(demolisher_x)
                    # gamelib.debug_write("combo : {} + {}".format(scout_num, demolisher_num))        # NOTE: for debugging
                    tmp_state = deepcopy(sim_state)
                    for i in range(scout_num):
                        tmp_state.game_map.add_unit(SCOUT, scout_location, 0)
                    for i in range(demolisher_num):
                        tmp_state.game_map.add_unit(DEMOLISHER, demolisher_location, 0)
                    # gamelib.debug_write("completed fake adding")        # NOTE: for debugging
                    # import time; tmp = time.time()
                    location_list = []
                    if scout_num > 0:
                        location_list.append(scout_location)
                    if demolisher_num > 0:
                        location_list.append(demolisher_location)
                    simulation = self.multi_attack_path_analysis(tmp_state, location_list)
                    # tmp = time.time() - tmp
                    # gamelib.debug_write("time elapsed: {} with combo: {} + {}".format(tmp, scout_num, demolisher_num))        # NOTE: for debugging
                    # gamelib.debug_write("simulation done")        # NOTE: for debugging
                    points = simulation[3]
                    damage = simulation[2]
                    if points > best_opportunity_pts[0]:
                        best_opportunity_pts = [points, opening, scout_x, demolisher_x, scout_num, demolisher_num]
                    if damage > best_opportunity_dmg[0]:
                        best_opportunity_dmg = [damage, opening, scout_x, demolisher_x, scout_num, demolisher_num]
            # choose points-first or damage-first
            if best_opportunity_pts[0] < 4:                                                     # TODO: Tunable
                best_opportunity = best_opportunity_dmg
            elif best_opportunity_dmg[0] < 400:
                best_opportunity = best_opportunity_pts
            elif best_opportunity_pts[0] * 80 > best_opportunity_dmg[0]:
                best_opportunity = best_opportunity_pts
            else:
                best_opportunity = best_opportunity_dmg
        else:
            best_opportunity = self.reserved_opportunity
        gamelib.debug_write("Best deployment strategy: {}".format(best_opportunity))        # NOTE: for debugging
        # if no opening, create one and deploy next turn
        if no_opening:
            self.reserved_opening = [best_opportunity[1]]
            self.reserved_opportunity = best_opportunity
            # gamelib.debug_write("reserved_opening: {}".format(self.reserved_opening))        # NOTE: for debugging
            game_state.attempt_remove(self.reserved_opening)
            # gamelib.debug_write("remove status: {}".format(success))        # NOTE: for debugging
            self.ready_to_attack = True
            gamelib.debug_write("Planned attack for next turn")        # NOTE: for debugging
            return
        # if has opening, spawn units
        if self.ready_to_attack and len(self.reserved_opening):
            if self.reserved_opening[0][0] < 13.5:
                game_state.attempt_spawn(INTERCEPTOR, self.get_edge_location(9), 1)
            else:
                game_state.attempt_spawn(INTERCEPTOR, self.get_edge_location(18), 1)
        game_state.attempt_spawn(DEMOLISHER, self.get_edge_location(best_opportunity[3]), best_opportunity[5])
        game_state.attempt_spawn(SCOUT, self.get_edge_location(best_opportunity[2]), best_opportunity[4])
        self.ready_to_attack = False
        self.reserved_opening = []
        self.reserved_opportunity = [-1, [-1, -1], -1, -1, -1, -1]

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
            elif dem_acm_num > 0:
                nums.append(dem_acm_num)
                units.append(game_state.game_map[[coord[0], coord[1]]][dem_pos])
            elif inter_acm_num > 0:
                nums.append(inter_acm_num)
                units.append(game_state.game_map[[coord[0],coord[1]]][inter_pos])
        
        
        pindex = units[0].player_index
        #gamelib.debug_write("player index: {}".format(pindex))
        #gamelib.debug_write("RW: number of different units deployed: {}".format(len(units)))
        #gamelib.debug_write("RW: number of deployment units: {}".format(len(nums)))
        can_reach = 0   
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
                        normal_supp_set.add(game_state.game_map[[j,i]][0])
                    else:
                        upgrade_supp_set.add(game_state.game_map[[j,i]][0])
        
        #gamelib.debug_write("RW: support set created")
        
        target_edges = []
        for loc in locations:
            target_edges.append(game_state.get_target_edge(loc))
        paths = []
        
        for i in range(len(units)):
            paths.append(game_state.find_path_to_edge(locations[i], target_edges[i]))
        
        #gamelib.debug_write("path length:{}".format(len(paths[0])))

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

        #gamelib.debug_write("adjusted path length:{}".format(len(adj_paths[0])))
        if len(adj_paths) <= 1:
            return can_reach, _dem_, _dmg_, _pnt_, _loss_, first_attack_pos, breach_pos, cross_bd_pos

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
            update_dmg = set()
            terminate_flag = 1
            dem_flag = 0
            for i, adj_path in enumerate(adj_paths):
                if adj_path == None:
                    del units[i]
                    del nums[i]
                    del adj_paths[i]
            for i, adj_path in enumerate(adj_paths):
                if ind < len(adj_path):
                    terminate_flag = 0
            if terminate_flag == 1:
                break
            
            for i, adj_path in enumerate(adj_paths):
                # This path has been exhausted
                if ind >= len(adj_paths[i]):
                    total_num -= nums[i]
                    nums[i] = 0
                    continue
                
                cur_pos = adj_path[ind] # current coordinate
                #gamelib.debug_write("current position: {}".format(cur_pos))
                if cur_pos[1] == 14:
                    cross_bd_pos[i] = cur_pos

                # Step 0: check if we reach the target edge
                edge_locations = game_state.game_map.get_edge_locations(target_edges[i])
                if cur_pos in edge_locations:
                    can_reach = 1
                    breach_pos[i] = cur_pos
                    _pnt_ += nums[i]
                    total_num -= nums[i]
                    nums[i] = 0
                    continue

                #gamelib.debug_write("RW: single path loop step 0 finished, seems no points scored, all good")

                # Step 1: Add shields through nearby supports
                # normal support
                temp_nss = []
                for nss in list_normal_supp_set[i]:
                    if nss.shieldRange > self.euclid_dist([nss.x, nss.y], cur_pos):
                        units[i].health += 3
                        temp_nss.append(nss)
                        _dmg_ += 10 * nums[i]
                for nss in temp_nss:
                    list_normal_supp_set[i].remove(nss)
                # upgraded support
                temp_uss = []
                for uss in list_upgrade_supp_set[i]:
                    if uss.shieldRange > self.euclid_dist([uss.x, uss.y], cur_pos):
                        units[i].health += 5 + 0.3 * uss.y
                        temp_uss.append(uss)
                        _dmg_ += 25 * nums[i]
                for uss in temp_uss:
                    list_upgrade_supp_set[i].remove(uss)
                        
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
                # For any unit, add the towers who are attacking this unit into update_dmg
                attacking_units = game_state.get_attackers([units[i].x, units[i].y], 1-pindex)
                if len(attacking_units) > 0:
                    for un in attacking_units:
                        if un not in update_dmg:
                            update_dmg.add(un)
                #gamelib.debug_write("RW: single path loop step 3 finished, all damage taken has been processed, all good")
        
            # For loop is over
            # First update the game_state
            tur_attack_units = {}
            for tur in update_dmg:
                att_unit = game_state.get_target(tur)
                if att_unit in tur_attack_units:
                    tur_attack_units[att_unit] += tur.damage_i
                else:
                    tur_attack_units[att_unit] = tur.damage_i
            for i, un in enumerate(units):
                if un in tur_attack_units:
                    if units[i].health <= tur_attack_units[un]:
                        nums[i] -= 1
                        total_num -= 1
                        _loss_ += 1
                        # 刷新，下一个 mobile unit 出现
                        if nums[i] > 0:
                            units[i].health = full_health[i]
                    else:
                        units[i].health -= tur_attack_units[un]
            for loc in update_SP:
                if game_state.game_map[[loc[0],loc[1]]] == []:
                    continue
                if game_state.game_map[[loc[0],loc[1]]][0].health > update_SP[loc]:
                    game_state.game_map[[loc[0],loc[1]]][0].health -= update_SP[loc]
                    _dmg_ += update_SP[loc]
                else:
                    _dmg_ += game_state.game_map[[loc[0],loc[1]]][0].health
                    game_state.game_map[[loc[0],loc[1]]] = []
                    _dem_ += 1
                    dem_flag = 1
            # Then update the adj_paths
            for i, path in enumerate(adj_paths):
                if ind >= len(path):
                    continue
                if nums[i] == 0:
                    continue
                if dem_flag == 0:
                    continue
                temp_path = game_state.find_path_to_edge([units[i].x, units[i].y], target_edges[i])
                temp_path = self.expand_path(temp_path, units[i].unit_type)
                #gamelib.debug_write("temp path : {}".format(temp_path))
                if len(temp_path) > 0:
                    cut_off = -1
                    while (cut_off > -1 * len(adj_paths[i]) and adj_paths[i][cut_off] == adj_paths[i][cut_off-1]):
                        cut_off -= 1
                    adj_paths[i] = adj_paths[i][:ind+cut_off+1] + temp_path
                    #gamelib.debug_write("adj path : {}".format(adj_paths[i]))
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
        #gamelib.debug_write("total path length: {}".format(ind))
        
        #gamelib.debug_write("Simulation over: can_reach:{}    _dem_:{}    _dmg_:{}  _pnt_:{}    _loss_:{}".format(can_reach, 
        #_dem_, _dmg_, _pnt_, _loss_))
        return can_reach, _dem_, _dmg_, _pnt_, _loss_, first_attack_pos, breach_pos, cross_bd_pos
        

    """
    ----------------------------helper functions----------------------------
    """
    
    def get_edge_location(self, x):
        # return edge coordinates given x
        return [x, math.floor(abs(13.5-x))]

    def get_enemy_turrets_on_board(self, game_state):
        count = 0
        for x in range(28):
            for y in range(14, 28):
                if not game_state.game_map.in_arena_bounds([x, y]): continue
                units = game_state.game_map[[x, y]]
                if len(units) == 0: continue
                if units[0].unit_type == TURRET:
                    count += 1
        return count
    
    def euclid_dist(self, loc1, loc2):
        dist = math.sqrt(pow(loc1[0] - loc2[0], 2) + pow(loc1[1] - loc2[1], 2))
        return dist
    
    def expand_path(self, path, utype):
        apath = []
        if utype == SCOUT:
            return path
        elif utype == DEMOLISHER:
            for loc in path:
                for i in range(2):
                    apath.append(loc)
            return apath
        else:
            for loc in path:
                for i in range(4):
                    apath.append(loc)
            return apath

if __name__ == "__main__":
    algo = AlgoStrategy()
    algo.start()
