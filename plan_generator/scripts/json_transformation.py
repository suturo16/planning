types = {'grasp-tool': '\"grasp\"', 'detach-tool-from-rack': '\"detach\"', 'hold-next-to-cake': '\"move-with-arm\"', 'place-plate-on-turtlebot': '\"move-with-arm\"',
	 'cut-cake': '\"cut\"', 'place-piece-of-cake-on-plate': '\"move-n-flip\"', 'place-spatula-on-table': ('\"move-with-arm\"', '\"move-gripper\"', '\"release\"')}

targets = {'hold-next-to-cake': '\"next2cake\"', 'place-plate-on-turtlebot': '\"deliver\"', 'place-spatula-on-table': ('\"spatulaDropZone\"', '\"open\"')}

arms = {'left': '\"r\"', 'right': '\"l\"'}

objects = {'knife': '\"cakeKnife\"', 'spatula': '\"cakeSpatula\"', 'plate': '\"dinnerPlateForCake\"', 'cake': '\"box\"'}

base_pose = "[{type: \"base-pose\"}"
base_pose_right = "[{type: \"base-pose\", arm: \"r\"}"


def transform_plan_to_json_string(plan):
    json_representation = base_pose
  
    actions = plan.split(";")[0].split('\n')
    actions = filter(None, actions)
    
    for action in actions:
        action = action.replace('(', '').replace(')','').split()
            
        if (action[0] == 'grasp-tool' or action[0] == 'detach-tool-from-rack'):
            json_representation = json_representation + "," + create_json_string_with_object(types[action[0]], arms[action[1]], objects[action[2]])
            if (action[2] == 'spatula'):
	        json_representation = json_representation + "," + base_pose
       
        elif (action[0] == 'hold-next-to-cake' or action[0] == 'place-plate-on-turtlebot'):
            json_representation = json_representation + "," + create_json_string_with_object_target(types[action[0]], arms[action[1]], objects[action[2]], targets[action[0]]) 
        
        elif (action[0] == 'cut-cake'):
            json_representation = json_representation + "," + create_json_string_with_cake_knife_target(types[action[0]], arms[action[1]], objects[action[2]], objects[action[3]], objects[action[4]]) + base_pose_right
    
        elif (action[0] == 'place-piece-of-cake-on-plate'):
            json_representation = json_representation + "," + create_json_string_with_tool_target(types[action[0]], arms[action[1]], objects[action[2]], objects[action[3]]) 

        elif (action[0] == 'place-spatula-on-table'):
             json_representation = json_representation + "," + create_json_string_with_three_types(types[action[0]][0], types[action[0]][1], types[action[0]][2], arms[action[1]], objects[action[2]], targets[action[0]][0], targets[action[0]][1])
    
        else:
            print ("Action " + str(action[0]) + " does not exist!")
    
    return json_representation + "]"


# (grasp-tool knife0 left) & (detach-tool-from-rack knife0 left)
def create_json_string_with_object(action_type, arm, object_used):
    s = "{{type: {}, arm: {}, object: {}}}"
    return s.format(action_type, arm, object_used)

# (hold-next-to-cake spatula0 right cake0) & (place-plate-on-turtlebot plate0 right)
def create_json_string_with_object_target(action_type, arm, object_used, target):
    s = "{{type: {}, arm: {}, object: {}, target: {}}}"
    return s.format(action_type, arm, object_used, target)

# (cut-cake knife0 left spatula0 right cake0 pieceofcake0)  
def create_json_string_with_cake_knife_target(action_type, arm, knife, cake, target):
    s = "{{type: {}, arm: {}, knife: {}, cake: {}, target: {}}}"
    return s.format(action_type, arm, knife, cake, target)

# (place-piece-of-cake-on-plate pieceofcake0 plate0 spatula0 right)  
def create_json_string_with_tool_target(action_type, arm, tool, target):
    s = "{{type: {}, arm: {}, tool: {}, target: {}}}"
    return s.format(action_type, arm, tool, target)

# (place-spatula-on-table spatula0 right)  
def create_json_string_with_three_types(type1, type2, type3, arm, object_used, target1, target2):
    s = "{{type: {}, arm: {}, object: {}, target: {}}},{{type: {}, arm: {}, target: {}}},{{type: {}, arm: {}}}"
    return s.format(type1, arm, object_used, target1, type2, arm, target2, type3, arm)
 
 
if __name__ == "__main__":
 
    plan = """(grasp-tool left knife)
(detach-tool-from-rack left knife)
(grasp-tool right spatula)
(cut-cake right knife cake spatula left pieceofcake0)
; cost = 9 (unit cost)"""

    print(transform_plan_to_json_string(plan))
